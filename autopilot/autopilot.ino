/*******************************************************
 * 
 * Arduino Wheele Auto Pilot for Scanmar 33
 * David Skogan, September 2013 - September 2015
 * 
 ********************************************************/

#include <L3G.h>
#include <Wire.h>
#include <runningAverageInt.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <LiquidCrystal.h>

/* DEFINITIONS */

#define version a4
#define date 2015-09-10
#define DEBUG false
#define flipMotor 1

// MOTOR CONTROLLER

// Motor read/transmit pins
#define moRxPin 12  // BRUN
#define moTxPin 11  // GRØNN

SoftwareSerial smcSerial(moRxPin, moTxPin); // Simple motor controller

// Motor controller
// GUL TX   --  moRxPin 11 - GRØNN
// GRØNN RX -- moTxPin  12 - BRUN

// Motor
// some variable IDs
#define ERROR_STATUS 0
#define LIMIT_STATUS 3
#define TARGET_SPEED 20
#define INPUT_VOLTAGE 23
#define TEMPERATURE 24
 
// some motor limit IDs
#define FORWARD_ACCELERATION 5
#define REVERSE_ACCELERATION 9
#define DECELERATION 2


// GPS

// Gps read/transmit pins
#define GPS_RX_PIN 3 // BLÅ
#define GPS_TX_PIN 2 // GUL

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPS gps;

boolean gpsSignal = false; // true if signal is recieved
long gpsTimestamp = 0; // time of last gps course signal
long gpsPrevCourse = 0; // previous gps cource
RunningAverageInt gpsAverageTurnRate(10); // average turn rate reported from gps

// GYRO
L3G gyro;

RunningAverageInt gyroAverageTurnRatePrSec(10);
RunningAverageInt gyroAverageTurnRatePrMin(10);
long gyroZ = 0; // beregnes til nullverdi

// LCD

// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the panel and buttons
int lcd_key     = 0;
int adc_key_in  = 0;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

#define RIGHT 0
#define LEFT 1
#define OFF 2

long lastDebounceTime = 0;  // the last time a key was toggled
long debounceDelay = 200;    // the debounce time
int lastButton = btnNONE;

// AUTOPILOT 

long course100 = 0; // current course (from gps) (scale x100)
int speed = 0;  // current speed (from gps)
int targetCourse = 0; // the course to stear
int prevTargetCourse = 0;

int motorDirection = OFF; // the direction of the motor
int prevDirection = 0; // previous direction of motor <0 left, >0 right

long loopTime = 1; // the time in milis that main controll routine takes. Should not be zero!

#define stSET   0
#define stON    1
#define stTRACK 2

#define apOFF           0
#define apCourseKeeping 1 
#define apCourseChange  2

int state = stSET;  // Menu state
int prevState = stSET;
int increment = 0;  // Size of the value to increment or decrement the target course

long turnRate100 = 0;  // Turn rate in degrees per second * 100
long accumulatedTurn = 0;
long turnGoal = 0;

int turnSector = 0 ;
int turn = 0;

int apState = apOFF;  // Autopilot state
int apSensitivity = 2;
int error = 0;


// required to allow motors to move
// must be called when controller restarts and after any error
void exitSafeStart()
{
  smcSerial.write(0x83);
}
 
// speed should be a number from -3200 to 3200
void setMotorSpeed(int speed)
{
  
  if (speed < 0)
  {
    smcSerial.write(0x86);  // motor reverse command
    speed = -speed;  // make speed positive
  }
  else
  {
    smcSerial.write(0x85);  // motor forward command
  }
  smcSerial.write(speed & 0x1F);
  smcSerial.write(speed >> 5);
}

// brake motor
void brakeMotor() {
    smcSerial.write(0x92);  // brake command
    smcSerial.write((byte)16);  // amount 0 - 32 (100%)

    delay(5);
      
    smcSerial.write(0x92);  // brake command
    smcSerial.write((byte) 0);  // amount 0 - 32 (100%)
}

// read the buttons
int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the sensor 
//Serial.println(adc_key_in);
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  // For V1.1 us this threshold
  /*
if (adc_key_in < 50)   return btnRIGHT;  
   if (adc_key_in < 250)  return btnUP; 
   if (adc_key_in < 450)  return btnDOWN; 
   if (adc_key_in < 650)  return btnLEFT; 
   if (adc_key_in < 850)  return btnSELECT;  
   */
  // For V1.0 comment the other threshold and use the one below:

  if (adc_key_in < 50)   return btnRIGHT;  
  if (adc_key_in < 195)  return btnUP; 
  if (adc_key_in < 380)  return btnDOWN; 
  if (adc_key_in < 555)  return btnLEFT; 
  if (adc_key_in < 790)  return btnSELECT;   

  return btnNONE;  // when all others fail, return this...
}



// 

void setup()
{
  Serial.begin(115200);
  Serial.println("Arduino Wheele Autopilot Starting");

  // Connect to the motor controller
  // initialize software serial object with baud rate of 19.2 kbps
  smcSerial.begin(19200);
 
  // the Simple Motor Controller must be running for at least 1 ms
  // before we try to send serial data, so we delay here for 5 ms
  delay(5);
 
  // if the Simple Motor Controller has automatic baud detection
  // enabled, we first need to send it the byte 0xAA (170 in decimal)
  // so that it can learn the baud rate
  smcSerial.write(0xAA);  // send baud-indicator byte

  // next we need to send the Exit Safe Start command, which
  // clears the safe-start violation and lets the motor run
  exitSafeStart();  // clear the safe-start violation and let the motor run
  Serial.println("Exit safe motor state!");
  
  Serial.println((int) getVariable(ERROR_STATUS), HEX);
//  Serial.println((int) getVariable(TEMPERATURE));


  lcd.begin(16, 2);              // start the LCD library
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Wheele Autopilot");
  lcd.setCursor(2,1);
  lcd.print(" starting...");


  // start communication with gyro

  Wire.begin();  

  if (!gyro.init()) {
        Serial.println("Failed to autodetect gyro type!");
  }
  
  gyro.enableDefault();
  
  // Compute Gyro Zero  
  int husk = 0;
  for (int i=0; i < 10; i++) {
    gyro.read();
    husk += gyro.g.z;
    delay(100);
  }
  
  gyroZ = husk / 10;
 
  // connect to the GPS
  gpsSerial.begin(4800); // STANDARD NMEA 0183
  //gpsSerial.begin(38400); // NMEA 0183 HS 38,400
    

  delay(200);
  lcd.clear();
  
}

// LCD


void printCourse()
{ 
  // TARGET COURSE ( 
  lcd.setCursor(5,0);
  targetCourse = normalizeCourse(targetCourse);  
 
  if (targetCourse < 100) {
      lcd.print(0);
  }
  if (targetCourse < 10) {
    lcd.print(0);
  }
  lcd.print(targetCourse); // lcd.print((char)223); //degrees
  
  // GPS SIGNAL DETECTED?
  if (!gpsSignal) {
     lcd.print("?"); 
  } else {
      lcd.print(" ");
  }
  
  // ERROR
  // print error
  lcd.setCursor(12,0);
  if (error != 0) {
    if (abs(error) < 100) { lcd.print(" "); }
    if (abs(error) < 10) {lcd.print(" "); }
    if (error > 0 ) { lcd.print("+"); } // 
    if(error != 0) lcd.print(error);
  } else { lcd.print("     "); }
  
  // INCREMENT
  lcd.setCursor(0,1);
  lcd.print(" -/+ ");
  printInt(increment);
  //lcd.print("  ");
}

void printInt(int i) {
    if (abs(i) < 100) { lcd.print(" "); }
    if (abs(i) < 10) {lcd.print(" "); }
    lcd.print(i);  
}

void printState()
{

  lcd.setCursor(0,0);
  switch (state)
  {
  case stSET:
    {
      lcd.print("SET:");
      break;
    }
  case stON:
    {
      lcd.print("ON :");
      break;
    }
  case stTRACK:
    {
      lcd.print("TRK:");
      break;
    }
  }
}

void printTurnRate() {

  int r = 0;
  if (turnRate100 != 0 ) r = turnRate100 / 100;
  lcd.setCursor(9,1);
  if (abs(r) < 100) { lcd.print(" "); }
  if (abs(r) < 10) {lcd.print(" "); }
  if (r > 0 ) { lcd.print("+"); } // 
  if (r != 0) {
    lcd.print(r);
  } else {
    lcd.setCursor(11,1);
    lcd.print("  ");
  }
}

void printMotor() {
  lcd.setCursor(14,1);
  if (motorDirection == RIGHT) {
    lcd.print("=>");
  } else if (motorDirection == LEFT) {
    lcd.print("<=");
  } else {
    lcd.print("[]");
  }
}

int normalizeCourse(int c) {
  int r = c;
  while (r < 0)   { 
    r = 360 + r; 
  }
  while (r > 359) { 
    r = r - 360; 
  }  

  return r;
}

/* Computes the smallest sector between the current course and the targetCourse
 * @param targetCourse
 * @return
 */
int sector(int targetCourse) {
  int lc = targetCourse - course100 / 100;
  int rc = 181;

  if (lc <= -180) rc = 360 + lc;
  if (lc >= 180) rc = lc - 360;

  if (abs(lc) < abs(rc)) {
    return lc;
  } 
  else {
    return rc;
  }
}

/* Check if there is any GPS sentences to read
*/
static bool feedgps()
{
  while (gpsSerial.available())
  {
    if (gps.encode(gpsSerial.read())) // gps encode returns true if a valid sentence has been read
      return true;
  }
  return false;
}

boolean firstTime = true;
  
/**
 * MAIN AUTOPILOT LOOP
 */
void loop()
{
  long start = millis();

  // read gps
  if (feedgps())
  { // new valid gps sentence have been processed. It can either be a RMC or a GGA. only RMC updates course

      // period since last timestamp
      long period = millis() - gpsTimestamp;

      if (gps.sentence_type() == TinyGPS::_GPS_SENTENCE_GPRMC) {
        course100 = gps.course();
        if (course100 != TinyGPS::GPS_INVALID_F_ANGLE) {
          //speed = gps.speed();
    
          //Serial.print("gps course: "); Serial.print(course);Serial.print(" update period: "); Serial.println(period);
          int gpsTurnRate = 0;
          if (!firstTime) gpsTurnRate = gpsPrevCourse - course100;
          firstTime = false;
          
          gpsAverageTurnRate.addValue(gpsTurnRate);
          
          // remember values to next update
          gpsPrevCourse = course100;
          gpsTimestamp = millis();          
        }
      }
      
     if (period > 10000 ) { // if gps signal is lost more than 10 seconds we should use gyro
       gpsSignal = false;
     } else {
       gpsSignal = true;
     }
  }
  

  // read rateGyro    
  gyro.read();
  long gz = (long) gyro.g.z;
  long trateMdps100 = (gz - gyroZ) * 875; // 8.75 will get mdps, 875 gives mdps x 100, 
  turnRate100 = trateMdps100 / 1000; // compute turn rate dps x 100
  accumulatedTurn += turnRate100 / 10; // er denne i bruk???
  
  // adjust course according to current turn rate.
  course100 += turnRate100 / 10 ;
  

  // display LCD
  printState();
  printCourse();
  printTurnRate();
  printMotor();

  // read buttons
  readButtons();

  // automatic update target course
  if (increment == 0 && state == stSET) {
    targetCourse = course100 / 100;
  } 

  // set course to stear
  if (state == stON ) {
    if (prevState == stSET) {
      setCourseToStear(targetCourse);
    } 
    else if (prevTargetCourse != targetCourse) {
      setCourseToStear(targetCourse);
    }
  }

  // make sure the motor is turned off when state is SET
  if (state == stSET && motorDirection != OFF) {
    motorOff();
  }

 // Serial.println((int)getVariable(TARGET_SPEED));
  
  error = sector(targetCourse); // - course100;
  
  // call stear procedure if ON 
  if (state == stON) {
    stear();
  }

  // update previous variables
  prevTargetCourse = targetCourse;
  prevState = state;

  // Try to run 10 loops a second 
  loopTime = millis() - start;
  if (loopTime < 100) {
    delay(100 - loopTime);
    // Serial.println(loopTime);
  } else {
    if (DEBUG) {
    Serial.print("loop=");
    Serial.println(loopTime);
    }
  }
  loopTime = millis() - start;
} // END LOOP

int buttonPressedLast = 0;

void readButtons() {
  lcd_key = read_LCD_buttons();  // read the buttons  
  long time = millis();

  if ( lcd_key == btnSELECT && buttonPressedLast == lcd_key) {
    // ignore select
    return;
  }

    // If the button changed
  if ( lcd_key != lastButton) {
    // reset the debouncing timer
    lastDebounceTime = time;
  } 

  if ((time - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    switch (lcd_key)               // depending on which button was pushed, we perform an action
    {
    case btnRIGHT:
      {
        targetCourse += increment;
        break;
      }
    case btnLEFT:
      {
        targetCourse -= increment;
        break;
      }
    case btnUP:
      {
        if (increment == 0) {
          increment = 1;
        }
        else if (increment == 1) {
          increment = 10;
        }
        else if (increment == 10 ) {
          increment = 100;
        }
        break;
      }
    case btnDOWN:
      {
        if (increment == 100) {
          increment = 10;
        }
        else if (increment == 10 ) {
          increment = 1;
        }
        else if (state == stSET) { 
          increment = 0; 
        }
        break;
      }
    case btnSELECT:
      {
        if (state == stON ) { 
          state = stSET; 
        }
        else { 
         state = stON; 
        }

        break;
      }
    case btnNONE:
      {
        break;
      }         
    }
    targetCourse = normalizeCourse(targetCourse);
    // reset debounce time to wait for next key
    lastDebounceTime = time; 
    buttonPressedLast = lcd_key;
    //lcd_key = 0; // to reset the lcd_key    
  }                   
  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButton = lcd_key;

}

void motorOff() {
   if (motorDirection != OFF) {
      motorDirection = OFF;
      setMotorSpeed(0);
    }
}

/**
 * Controls the motor
 * @param direction 
 * @param speed 
 */
void motorOn(int direction, int speed) {

  
  if (speed > 0) {
    int motor_speed = flipMotor * 3200 * speed / 100;
    
    if (direction == RIGHT) {
      // set motor on right
      if (motorDirection != RIGHT) {
        motorDirection = RIGHT;
        setMotorSpeed(motor_speed);
      }
    } 
    else if (direction == LEFT) {
      // set motor on left
      if (motorDirection != LEFT) {
        motorDirection = LEFT;
        setMotorSpeed(-motor_speed);
      }
    }
  } 
  else {
    // set motor off
    if (motorDirection != OFF) {
      motorDirection = OFF;
      setMotorSpeed(0);
    }
    
  }
  
  prevDirection = direction;
}



// read a serial byte (returns -1 if nothing received after the timeout expires)
int readByte()
{
  char c;
  if(smcSerial.readBytes(&c, 1) == 0){ return -1; }
  return (byte)c;
}
 
unsigned char setMotorLimit(unsigned char  limitID, unsigned int limitValue)
{
  smcSerial.write(0xA2);
  smcSerial.write(limitID);
  smcSerial.write(limitValue & 0x7F);
  smcSerial.write(limitValue >> 7);
  return readByte();
}
 
// returns the specified variable as an unsigned integer.
// if the requested variable is signed, the value returned by this function
// should be typecast as an int.
unsigned int getVariable(unsigned char variableID)
{
  smcSerial.write(0xA1);
  smcSerial.write(variableID);
  return readByte() + 256 * readByte();
}
 

/**
 * Set the new target course
 * 
 * @param newCourse
 */
void setCourseToStear(int newCourse) {
  targetCourse = normalizeCourse(newCourse);
  turnSector = sector(targetCourse);

  accumulatedTurn = 0;
  turnGoal = 0;

  apState = apCourseKeeping;

  if (DEBUG) {
  Serial.print("Set: targetCourse= ");
  Serial.print(targetCourse);
  Serial.print(" turnSector= ");
  Serial.print(turnSector);
  if (turnGoal != 0) {
    Serial.print(" turnGoal = ");
    Serial.print(turnGoal);
  }
  Serial.println();
  }
}


int sign(long value) {

  if (value < 0)
    return -1;
  else return 1;  
}

void logit (int message, int error, int tickL, int tickR) {

  if (DEBUG) {
  if (message == RIGHT ) Serial.print(   "RIGHT ");
  else if (message == LEFT) Serial.print("LEFT  ");
  else Serial.print(                     "OFF   "); 
  Serial.print(error); 
  Serial.print(" ");
  Serial.print(tickL); 
  Serial.print(" ");
  Serial.print(tickR);
  Serial.println();
  }
}



int motorTime = 0;
int waitTime = 0;
int prevError = 0; // takes values between -180 and 180
long prevTurnRate = 0; // takes values between -32000 and 32000

/**
 * Main autopilot control routine. 
 * 
 *  Parametre: 

   error  - the error defined as (targetCourse - course) (but sectorized) (degrees). Example targetCourse=100 - course=120 = -20, i.e. we need to turn 20 degrees to the left.
   prevError - the error last time this loop was run
   turnRate100 - the turn rate in degrees per second * 100 
   motorDirection - the status of the motor (RIGHT, OFF, LEFT)
   motorTime - the time to run the motor
   waitTime - the time to wait before adjusting.
   
  */
int mState = 0;
int oldMState = 0;

void stear() {
  
  oldMState = mState;
  mState = getMotorState(error, turnRate100);
  
  if ( mState == 0 ) {
      motorOff();
      motorTime = 0;
  } else {
  
    int speed = ( abs(mState) * 50 ) / 4 + 50; // konverter tallene til % pådrag for motoren 4 - 100%, 3-87 2 75%, 1 - 62%
    
    if ( mState < 0 ) {  
      motorOn(LEFT, speed); 
      motorTime++;
    } else if ( mState > 0 ) {
      motorOn(RIGHT, speed); 
      motorTime++;
    }
  }
    // Safety - stop motor  if we have run the motor for more than 10 seconds
  if ( motorTime > 100 ) { 
    
    motorOff();
    motorTime = 0;
    delay(1000);    
    
    state = stSET; // stop course tracking
    lcd.setCursor(10,1);
    lcd.print("ERROR!");
  }  
  
  // remember last iteration
  prevError = error;
  prevTurnRate = turnRate100;
    
} // end stear

// Terskler. 5 områder for error og 5 for turnRate

int er[] = { -10, -2, +2, +10};
int tr[] = { -500, -80, +80, 500}; // turnRate er ganget med 100, dvs. -500 tilsvarer -5 grader per sekund

int getMotorState(int error, int turnRate) {

  // tr: --
  if (turnRate < tr[0] ) {
    if ( error < er[0] )                    return 0; // er: --
    if ( error < er[1] )                    return 1; // er: -
    if ( error >= er[1] && error <= er[2] ) return 2; // er: 0
    if ( error <= er[3] )                   return 3; // er: +
    if ( error > er[3] )                    return 4; // er: ++
  }
  
  // tr: -
  if (turnRate < tr[1]) {
    if ( error < er[0] )                    return -1;
    if ( error < er[1]  )                   return 0;
    if ( error >= er[1] && error <= er[2] ) return 1;
    if ( error <= er[3] )                   return 2;
    if ( error > er[3] )                    return 3;
  }
  
  // tr: 0
  if (turnRate >= tr[1] && turnRate <= tr[2] ) {
    if ( error < er[0] )                    return -2;
    if ( error < er[1] )                    return -1;
    if ( error >= er[1] && error <= er[2] ) return 0;
    if ( error <= er[3] )                   return 1;
    if ( error > er[3] )                    return 2;
  }
  
  // tr: +
  if (turnRate <= tr[3]) {
    if ( error < er[0] )                    return -3;
    if ( error < er[1] )                    return -2;
    if ( error >= er[1] && error <= er[2] ) return -1;
    if ( error <= er[3] )                   return 0;
    if ( error > er[3] )                    return 1;
  }
  
  // tr: ++
  if ( turnRate > tr[3] ) {
    if ( error < er[0] )                    return -4;
    if ( error < er[1] )                    return -3;
    if ( error >= er[1] && error <= er[2] ) return -2;
    if ( error <= er[3] )                   return -1;
    if ( error > er[3] )                    return 0;
  }

  
 } // getMotorState

