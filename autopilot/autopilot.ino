/*******************************************************

   Arduino Wheel Auto Pilot for Scanmar 33
   David Skogan, September 2013 - Oktober 2016

 ********************************************************/
/*
 * funn: 2016-10-08
 *  kurs kan bli minus - diff gps vs course100 - fix med normalizeCourse100
 *  turnRate100 er for stor for correctionFactor. Delte på 10 for å kompensere med at vi looper 10 ganger i sekundet
 *  fjernet running average og ubrukte variable
 * 
 */

#include <L3G.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

/* DEFINITIONS */

#define version a6
#define date 2016-10-08
#define LOG true
#define flipMotor 1

// MOTOR CONTROLLER
// Motor read/transmit pins
#define moRxPin 12  // BRUN
#define moTxPin 11  // GRØNN
int mState = 0; // Holds the curren motor state

SoftwareSerial smcSerial(moRxPin, moTxPin); // Simple motor controller

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

// GYRO
L3G gyro;
long gyroZ = 0; // the zero value of the gyro

// LCD
// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the panel and buttons
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

// defines motor direction
#define RIGHT 0
#define LEFT 1
#define OFF 2

int lastButton = btnNONE; // remembers the last button read

// AUTOPILOT
long course100 = 0; // current course (from gps + adjusted by gyro) (scale x100)
int speed = 0;  // current speed (from gps)
int targetCourse = 0; // the course to stear
int prevTargetCourse = 0;  // remembers the previous target course
int motorDirection = OFF; // the direction of the motor
long loopTime = 1; // the time in milis that main controll routine takes. Should not be zero!

// Controller Menu State
#define stSET   0 
#define stON    1
#define stTRACK 2

int state = stSET;  // Menu state
int prevState = stSET;
int increment = 0;  // Size of the value to increment or decrement the target course
long turnRate100 = 0;  // Turn rate in degrees per second * 100
int error = 0; // The error between the current course and the targetCourse
int motorTime = 0; // used to count how long the motor has been running

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
  int adc_key_in = analogRead(0);      // read the value from the sensor
  
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
  lcd.setCursor(0, 0);
  lcd.print("Wheele Autopilot");
  lcd.setCursor(2, 1);
  lcd.print("v1 starting...");

  // start communication with gyro
  Wire.begin();

  if (!gyro.init()) {
    Serial.println("Failed to autodetect gyro type!");
  }

  gyro.enableDefault();

  // Compute Gyro Zero
  int husk = 0;
  for (int i = 0; i < 10; i++) {
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
  lcd.setCursor(5, 0);
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
  lcd.setCursor(12, 0);
  if (error != 0) {
    if (abs(error) < 100) {
      lcd.print(" ");
    }
    if (abs(error) < 10) {
      lcd.print(" ");
    }
    if (error > 0 ) {
      lcd.print("+");  //
    }
    if (error != 0) lcd.print(error);
  } else {
    lcd.print("     ");
  }

}

void printInt(int i) {
  if (abs(i) < 100) {
    lcd.print(" ");
  }
  if (abs(i) < 10) {
    lcd.print(" ");
  }
  lcd.print(i);
}

void printState()
{
  lcd.setCursor(0, 0);
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
  lcd.setCursor(8, 1);
  
  if (abs(r) < 100) {
    lcd.print(" ");
  }
  if (abs(r) < 10) {
    lcd.print(" ");
  }
  if (r > 0 ) {
    lcd.print("+");  //
  }
  if (r != 0) {
    lcd.print(r);
  } else {
    lcd.setCursor(8, 1);
    lcd.print("    ");
  }
}

void printMotor() {
  lcd.setCursor(13, 1);
  if (motorDirection == RIGHT) {
    lcd.print("=>");
    lcd.print(abs(mState));
  } else if (motorDirection == LEFT) {
    lcd.print(abs(mState));
    lcd.print("<=");
  } else {
    lcd.print("[ ]");
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

long normalizeCourse100(long c) {
  int r = c;
  while (r < 0)   {
    r = 36000 + r;
  }
  while (r > 35999) {
    r = r - 36000;
  }

  return r;
}

/* Computes the smallest sector between the current course and the targetCourse
   @param targetCourse
   @return
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

boolean firstTime = true; // used to set the course first time gps signal is read

/**
*   MAIN AUTOPILOT LOOP
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
      long gpsCourse100 = gps.course();
      if (gpsCourse100 != TinyGPS::GPS_INVALID_F_ANGLE) {
        //speed = gps.speed();

        //Serial.print("gps course: "); Serial.print(course);Serial.print(" update period: "); Serial.println(period);
        if (firstTime) { // update course to match gps first time
          course100 = gpsCourse100;
          firstTime = false;
        }
        if (LOG) {
          Serial.print("gps: ");
          Serial.print(millis());
          Serial.print(" ");
          Serial.print(gpsCourse100);
          Serial.print(" ");
          Serial.print(course100);
          Serial.println();
        }
        course100 = gpsCourse100;
        
        //gpsAverageAdjustment.addValue(sector( gpsCourse100 / 100)); // adds the difference between current course and gps course, i.e. sector-error
        //course100 = course100 + gpsAverageAdjustment.getAverage() * 100;
        
        //gpsAverageAdjustment.getAverage();
        // remember values to next update
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

  // adjust course according to current turn rate. NormalizeCourse ensures that we newer get minus course
  course100 = normalizeCourse100( course100 + turnRate100 / 10 ) ;

  if (LOG) {
    Serial.print("gyr: ");
    Serial.print(millis());
    Serial.print(" ");
    Serial.print(gz);
    Serial.print(" ");
    Serial.print(turnRate100);
    Serial.print(" ");
    Serial.print(course100);
    Serial.println();
  }
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
    if (prevState == stSET || prevTargetCourse != targetCourse ) {
      setCourseToStear(targetCourse);
      if (LOG) {
        Serial.print("tac: ");
        Serial.print(millis());
        Serial.print(" ");
        Serial.print(targetCourse);
        Serial.println();
      }
    }
  }

  // make sure the motor is turned off when state is SET
  if (state == stSET && motorDirection != OFF) {
    motorOff();
  }

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
  } 
  
  loopTime = millis() - start;
} // END LOOP


int buttonCount = 0;
/*
 * READ BUTTONS
 * 
 */
void readButtons() {
  int lcd_key = read_LCD_buttons();  // read button

  if (lcd_key != btnNONE ) { 
    
  //Serial.print("key: ");
  //Serial.println(lcd_key);
  
  // If the button changed
  if ( lcd_key != lastButton) {
    buttonCount = 1;
  } else {
    buttonCount ++;
  }

  //if ((time - lastDebounceTime) > debounceDelay) {
  if ( lcd_key == lastButton && buttonCount >= 2) {
    buttonCount = 0;  
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    switch (lcd_key)  // depending on which button was pushed, we perform an action
    {
      case btnRIGHT:
        {
          targetCourse += 10;
          break;
        }
      case btnLEFT:
        {
          targetCourse -= 10;
          break;
        }
      case btnUP:
        {
          targetCourse -= 1;
          break;
        }
      case btnDOWN:
        {
          targetCourse += 1;
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
    
  }
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
   Controls the motor
   @param direction
   @param speed (value between 0 and 100)
*/
void motorOn(int direction, int speed) {

  if (speed > 0) {
    int motor_speed = flipMotor * 32 * speed ;  //flipMotor * 3200 * speed / 100;

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

}



// read a serial byte (returns -1 if nothing received after the timeout expires)
int readByte()
{
  char c;
  if (smcSerial.readBytes(&c, 1) == 0) {
    return -1;
  }
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
   Set the new target course

   @param newCourse
*/
void setCourseToStear(int newCourse) {
  targetCourse = normalizeCourse(newCourse);
}


int sign(long value) {

  if (value < 0)
    return -1;
  else return 1;
}


/**
   Main autopilot control routine.

    Parametre:

   error  - the error defined as (targetCourse - course) (but sectorized) (degrees). Example targetCourse=100 - course=120 = -20, i.e. we need to turn 20 degrees to the left.
   turnRate100 - the turn rate in degrees per second * 100
   motorDirection - the status of the motor (RIGHT, OFF, LEFT)
   motorTime - the time to run the motor
*/

void stear() {
  int tr = turnRate100 / 10;
  mState = correctionFactor(error, tr);

  //predict
  long ticks = (error * 100) / tr;

  if (LOG) {
    Serial.print("ste: ");
    Serial.print(millis());
    Serial.print(" ");
    Serial.print(error);
    Serial.print(" ");
    Serial.print(turnRate100);
    Serial.print(" ");
    Serial.print(mState);
    Serial.print(" ");
    Serial.print(ticks);
    Serial.println(); 
  }
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
  
  // Safety - stop motor  if we have run the motor for more than 2 minutes/120 seconds
  if ( motorTime > 1200 ) {

    motorOff();
    motorTime = 0;
    delay(1000);

    state = stSET; // stop course tracking
    lcd.setCursor(10, 1);
    lcd.print("ERROR!");
  }

  // remember last iteration
  //prevTurnRate = turnRate100;

} // end stear

/*
 * Terskler. 5 områder for error og 5 for turnRate
 * Følgende intervalltolkninger:
 * ( -- , -10) [10,-2) [2, 2] (2, 10] (10, ++) */
int er[] = { -10, -2, +2, +10};    // error is in degrees
int tr[] = { -100, -50, +50, 100}; // turnRate er ganget med 100, dvs. -100 tilsvarer -1 grader per tidels-sekund, 50 tilsvarer 0,5 grader per tidels-sekund

/* 
*  Main decision function. It first checks turnRate, then error 
*/
int correctionFactor(int error, int turnRate) {

  // tr: -- ( --, -500)
  if (turnRate < tr[0] ) {
    if ( error < er[0] )    return 0; // er: --
    if ( error < er[1] )    return 1; // er: -
    if ( error <= er[2] )   return 2; // er: 0
    if ( error <= er[3] )   return 3; // er: +
    if ( error > er[3] )    return 4; // er: ++
  }

  // tr: - (-500, -50)
  if (turnRate < tr[1]) {
    if ( error < er[0] )    return -1;
    if ( error < er[1] )    return 0;
    if ( error <= er[2] )   return 1;
    if ( error <= er[3] )   return 2;
    if ( error > er[3] )    return 3;
  }

  // tr: 0 [-50, 50]
  if (turnRate <= tr[2] ) {
    if ( error < er[0] )   return -2;
    if ( error < er[1] )   return -1;
    if ( error <= er[2] )  return 0;
    if ( error <= er[3] )  return 1;
    if ( error > er[3] )   return 2;
  }

  // tr: + (50, 500]
  if (turnRate <= tr[3]) {
    if ( error < er[0] )    return -3;
    if ( error < er[1] )    return -2;
    if ( error <= er[2] )   return -1;
    if ( error <= er[3] )   return 0;
    if ( error > er[3] )    return 1;
  }

  // tr: ++ (500, ++)
  if ( turnRate > tr[3] ) {
    if ( error < er[0] )     return -4;
    if ( error < er[1] )     return -3;
    if ( error <= er[2] )    return -2;
    if ( error <= er[3] )    return -1;
    if ( error > er[3] )     return 0;
  }

  return 0;
} // Motor correction factor

