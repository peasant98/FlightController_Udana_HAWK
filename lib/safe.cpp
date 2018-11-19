/* In case something goes wrong with the drone implementation of
everything, this code (which is the essentially) non-refactored
code that has the functionality desired.

*/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include "..\resources\Adafruit_Sensor.h"
#include "..\resources\Adafruit_LSM9DS1.h"
#include "..\resources\Adafruit_LSM9DS1.cpp"
#include <math.h>
// written by Matthew Strong
// Code for the Udana HAWK Drone

// first 'version' is based on a drone that can be controlled by a keyboard
#define Pi 3.1415926589793238
#define MIN_PULSE_LENGTH 1000
// Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000
// Maximum pulse length in µs
// in order to have stabilize mode, need to keep track of the current angle of the drone

// important values for keePINg track of things

float throttleValue, pitchValue, yawValue, rollValue;
float angleRollAcc, anglePitchAcc, anglePitch, angleRoll;
int highestThrottle = 1950;
// p, i, and d settings for the roll
float PgainRoll = 1.2;
float IgainRoll = 0.04;
float DgainRoll = 3.5;
// the d gain value could increase - by too much - the sensitivity of the drone - need to account for this
// maximum output of the controller
int pidMaxRoll = 400;

// p, i, and d settings for the pitch
float PgainPitch = PgainRoll;
float IgainPitch = IgainRoll;
float DgainPitch = DgainRoll;

// maximum output of the controller
int pidMaxPitch = pidMaxRoll;

// p, i, and d settings for the yaw
float PgainYaw = 4.0;
float IgainYaw = 0.02;
float DgainYaw = 0.0;



int rollAngle, pitchAngle, yawAngle;

int pidMaxYaw = pidMaxRoll;
// boolean leveling = true;
int accelerationX, accelerationY, accelerationZ, accelerationNet;
int throttle;

const float timerVal = (0.02);
// timer val
float batteryVoltage;
float pidErrorTemp;
float iMemRoll, iMemPitch, iMemYaw;
float rollSetpoint, pitchSetpoint, yawSetpoint;
float rollInputGyro, pitchInputGyro, yawInputGyro;
float pidRollOutput, pidPitchOutput, pidYawOutput;
float pidLastRoll_D_Error, pidLastPitch_D_Error, pidLastYaw_D_Error;
Servo esc1, esc2, esc3, esc4;
int esc1Value, esc2Value, esc3Value, esc4Value;
char data;
unsigned long timerDrone;
// possibly the time for the drone currently
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
// arrays for the roll, pitch and yaw for the keybpard
float rollCalibration, pitchCalibration, yawCalibration;
int startingCode;
float rollAdjust, pitchAdjust;
bool autoLeveling = true;
// some arrays to keep track of gyro and acceleration so far
// define any necessary PINs here
float gyroArr[4] = {0,0,0,0};
float accArr[4] = {0,0,0,0};
// declaring functions in advance
void test();
void displayInstructions();
void readData();
void calibrateESC();
void gyroCalibrations();
void routine();

unsigned int calcTimeSpan(int freq);

unsigned int del;
//Motor 1 : front left - clockwise
//Motor 2 : front right - counter-clockwise
//Motor 3 : rear left - clockwise
//Motor 4 : rear left - counter-clockwise
// this is the case with every drone we make!
// each one connected as an esc using the Servo library
void setup() {
    // put your setup code here, to run once:
    // the main code that is very similar to the arduino code
    Serial.begin(9600);
    // the primary sensor
    // seeing if the sensor works and is connected correctly
    if(!lsm.begin())
  {
    // issue with connecting
    Serial.println("Ooops, no LSM9DS1 detected ... Check your wiring!");
  }
  else{
    // setting up acceleration, magnetism and gyro
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
    //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
    // 2.) Set the magnetometer sensitivity
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
    //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
    // max dps is 245
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    //lsm.setupGyro(lsm.LS  M9DS1_GYROSCALE_500DPS);
    //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
    //PINMode(LED3, OUTPUT);
    //pinMode(LED2, OUTPUT);
    //pinMode(LED1, OUTPUT);
    //pinMode(LED0, OUTPUT);
    startingCode = 0;
    throttleValue = 1000.0;
    pitchValue = 0.0;
    yawValue = 0.0;
    rollValue = 0.0;
    // setting all of the intermediate values to calculate the pitch, roll, yaw, and throttle
    // need to calibrate the gyros when the drone is stationary
    // must be stationary or the drone will not work!!!
    rollCalibration = 0;
    pitchCalibration = 0;
    yawCalibration = 0;

    //Serial.println("Calibrating all of the gyro values to zero. Please have the drone stationary.");

    gyroCalibrations();
    // subtract that everything once we read in dps input for the gyro
    // subtract this from the current reading for the best relative value
    // will account for this as the drone is moving
    // getting the averages of everything thus far
    esc1.attach(11, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    esc2.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    esc3.attach(10, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    esc4.attach(12, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    // now for calbrating and arming the escs (one still doesn't work..)
    //calibrateESC();
    // requires the user to plug in the battery10
    // will allow the drone to be properly calibrated.
    // after this and the user has plugged in the drone we will be ready to begin
    // startingCode reset to account for this

    // comment out displayInstructions() function when testing keyboard input on Rasbperry Pi.
    //displayInstructions();

    startingCode = 0;
    // since the drone has not been started or calibrated at all yet
    //batteryVoltage = (analogRead(0) + 65) * 1.2317;
    rollAngle = 0;
    pitchAngle = 0;
    yawAngle = 0;
    pitchAdjust = 0;
    rollAdjust = 0;
    del = calcTimeSpan(50);
    timerDrone = micros();
    // type of micros is unsigned long here
    // amount in the battery
    // all steps before doing anything
  }
}
void displayInstructions()
{
    Serial.println("Welcome to the Udana HAWK Serial Console!");
    Serial.println("");
    Serial.println("Choose an Option:");
    Serial.println("Q - Increase the Throttle");
    Serial.println("W - Decrease the Throttle");
    Serial.println("A - Up the Pitch");
    Serial.println("S - Down the Pitch");
    Serial.println("Z - Up the Yaw");
    Serial.println("X - Down the Yaw");
    Serial.println("E - Up the Roll");
    Serial.println("R - Down the Roll");
    Serial.println("O - Stop the Drone (Don't do this in midair!)");
    Serial.println("P - Go, get the drone to begin again");
    Serial.println("0 : Send min throttle");
    Serial.println("1 : Send max throttle");
}


// pid algorithm: once we have the desired user inputs we can calculate what pulse we must send to each of the motors
// the input for the function will be in degrees per second
// note that throttle is independent of these values, that really determines how far up the drone goes, not its x, y, or z orientation and desired
// direction.
void findPID(){
  // the pid will be messed around when the rollInputGyro is fluctuating rapidly
  // when the throttle is low, even small variations in pidErrorTemp cause fluctuation in the motors at this speed
  // and the differences become proportionally less than the desired throttle, at a higher throttle
  pidErrorTemp = rollInputGyro - rollSetpoint;
  // difference in the data reading and roll setpoint, the dps that the drone needs to go
  // difference between the current dps and dps that the drone needs to get to
  // calculating the big boy, the p, which accounts for the most work
  // roll error, input - what we actually want
  iMemRoll += IgainRoll * pidErrorTemp;
  // accounting for too high intergral roll
  if(iMemRoll > pidMaxRoll){
    iMemRoll = pidMaxRoll;
  }
  else if(iMemRoll < (pidMaxRoll*-1)){
    iMemRoll = (pidMaxRoll * -1);
  }
  pidRollOutput = PgainRoll * pidErrorTemp + iMemRoll + DgainRoll * (pidErrorTemp - pidLastRoll_D_Error);

  if(pidRollOutput > pidMaxRoll){
    pidRollOutput = pidMaxRoll;
  }
  else if(pidRollOutput < (pidMaxRoll * -1)){
    pidRollOutput = (pidMaxRoll * -1);
  }
  pidLastRoll_D_Error = pidErrorTemp;
  // end of pitch. If we lived in a one-dimensional world, we would be done.
  // but alas, we are not
  pidErrorTemp = pitchInputGyro - pitchSetpoint;
  iMemPitch += IgainPitch * pidErrorTemp;
  // the integral pitch is the gain multiplied by the error so far and the gain, which is set manually and has to be tuned
  if(iMemPitch > pidMaxPitch){
    iMemPitch = pidMaxPitch;
  }
  else if(iMemPitch < (pidMaxPitch*-1)){
    iMemPitch = (pidMaxPitch * -1);
  }
  // this is affected a lot
  pidPitchOutput = PgainPitch * pidErrorTemp + iMemPitch + DgainPitch * (pidErrorTemp - pidLastPitch_D_Error);

  if(pidPitchOutput > pidMaxPitch){
    pidPitchOutput = pidMaxPitch;
  }
  else if(pidPitchOutput < (pidMaxPitch * -1)){
    pidPitchOutput = (pidMaxPitch * -1);
  }
  pidLastPitch_D_Error = pidErrorTemp;

  pidErrorTemp = yawInputGyro - yawSetpoint;
  iMemYaw += IgainYaw * pidErrorTemp;
  if(iMemYaw > pidMaxYaw){
    iMemYaw = pidMaxYaw;
  }
  else if(iMemYaw < (pidMaxYaw*-1)){
    iMemYaw = (pidMaxYaw*-1);
  }
  //this is is affected a lot
  pidYawOutput = PgainYaw * pidErrorTemp + iMemPitch + DgainYaw * (pidErrorTemp - pidLastYaw_D_Error);
  if(pidYawOutput > pidMaxYaw){
    pidYawOutput = pidMaxYaw;
  }
  else if(pidYawOutput < (pidMaxYaw * -1)){
    pidYawOutput = (pidMaxYaw * -1);
  }
  pidLastYaw_D_Error = pidErrorTemp;
  // the most important outputs are :
  // pidYawOutput
  // pidRollOutput
  // pidPitchOutput
}
//Calculate the pulse for esc 1 (front-right - CCW)
// my case: esc C
// PIN 11

//Calculate the pulse for esc 2 (rear-right - CW)
// my case: esc B
// PIN 9

//Calculate the pulse for esc 3 (rear-left - CCW)
// my case: esc A
// PIN 10

//Calculate the pulse for esc 4 (front-left - CW)
// my case: esc D
// PIN 12

  // the refresh rate of the 30A BLD escs is 50 - 60 Hz
  //escs need pulse every 1/50 = 20ms
  // higher refresh rate - 60  - 1/60 -> 16.6ms -> 17 ms
  // take higher value because of the uncertainty of the values, less cycles per second
void sendUnison(int uniValue){
  esc1.writeMicroseconds(uniValue);
  esc2.writeMicroseconds(uniValue);
  esc3.writeMicroseconds(uniValue);
  esc4.writeMicroseconds(uniValue);
}

float decrementAbsValue(float previous, float toChange){
  if(previous <0){
    toChange = toChange + 20;
  }
  else if(previous >0){
    toChange = toChange - 20;
  }
  else{
    toChange = 0;
  }
  return toChange;
}

void loop() {
  float prevThrottle = throttleValue;
  float prevPitch = pitchValue;
  float prevYaw = yawValue;
  float prevRoll = rollValue;
  bool notCalibrating = true;
  // serial will be available with the raspberry pi
  if (Serial.available()) {
        data = Serial.read();
        Serial.println(data, DEC);
        // switch statement more effective than numerous if statements
        switch (data){
          case 113:
            throttleValue+=4.0;
            notCalibrating = true;
            break;

          case 119:
            throttleValue-=4.0;
            notCalibrating = true;
            break;

          case 97:
            pitchValue+=25.0;
            notCalibrating = true;
            break;

          case 115:
            pitchValue-=25.0;
            notCalibrating = true;
            break;

          case 122:
            yawValue+=25.0;
            notCalibrating = true;
            break;

          case 120:
            yawValue-=25.0;
            notCalibrating = true;
            break;

          case 101:
            rollValue+=25.0;
            notCalibrating = true;
            break;

          case 114:
            rollValue-=25.0;
            notCalibrating = true;
            break;

          case 111:
            if(startingCode){
              // if the drone is in the starting condition we can bring it to a stop, not the other way around
              sendUnison(MIN_PULSE_LENGTH);
              startingCode = 0;
            }
            break;

          case 112:
            if(!startingCode){
              calibrateESC();
              // recalibrate esc because the starting code is now 0
              iMemRoll = 0;
              iMemPitch = 0;
              iMemYaw = 0;
              pidLastRoll_D_Error = 0;
              pidLastPitch_D_Error = 0;
              pidLastYaw_D_Error = 0;
              // resetting all of the pid values when everything is restarted.
              //Serial.println("Restarting the HAWK from being STATIONARY...");
              // the HAWK has already been calibrated at this point in the program though
              startingCode = 1;
            }
            break;

          case 48:
            sendUnison(MIN_PULSE_LENGTH);
            notCalibrating = false;
            anglePitch = anglePitchAcc;
            angleRoll = angleRollAcc;
            // conduct the testing and starting up the drone

            delay(5000);
            startingCode = 1;
            //startingCode = 1;
            break;

          case 49:
            sendUnison(MAX_PULSE_LENGTH);
            notCalibrating = false;
            break;
        }
        // roll - x
        // pitch - y
        // yaw - z, user can increment or decrement these values
    }
    if(notCalibrating and startingCode){
      // the drone must be at the startingCode of 1 to be able to be written a pulse to
      if(prevThrottle == throttleValue){
        throttleValue = prevThrottle;
      }
      if(prevPitch == pitchValue){
        // pitch was not changed, bring it down to the absolute value
        pitchValue  = decrementAbsValue(prevPitch, pitchValue);
      }
      if(prevYaw == yawValue){
        yawValue = decrementAbsValue(prevYaw, yawValue);
        // yaw was not changed, bring down to the absolute value
      }
      if(prevRoll == rollValue){
        rollValue = decrementAbsValue(prevRoll, rollValue);
        // roll was not changed, will stabilize back to 0, but not immediately
      }
      // below code ensures that the throttle, pitch, roll, and yaw do not go out of the bounds of the
      // max values - and that the drone does not tip over.
      if(throttleValue >= 1950){
        throttleValue = 1950;
      }
      else if(throttleValue <= (1000)){
        throttleValue = 1000;
      }
      if(pitchValue >= 500){
        pitchValue = 500;
      }
      else if(pitchValue <= -500){
        pitchValue = -500;
      }
      if(rollValue >= 500){
        rollValue = 500;
      }
      else if(rollValue <= -500){
        rollValue = -500;
      }
      if(yawValue >= 500){
        yawValue = 500;
      }
      else if(yawValue <= -500){
        yawValue = -500;
      }
      // after all of the edge cases, we can set the pidsetpoints to what the user has provided... then calibrate via the angle

      throttle = int(throttleValue);
      pitchSetpoint = pitchValue;
      yawSetpoint = yawValue;
      rollSetpoint = rollValue;
    }
    // pid input should be in degrees per second, gyro goes to +- 245 dp
    /*
    pitchSetpoint = pitchSetpoint - pitchAdjust;
    rollSetpoint = rollSetpoint - rollAdjust;
    */
    // input is subtracted from what the roll is expected at the stationary surface; that is, when all axes are at 0 ....
    rollInputGyro = gyroArr[0] - rollCalibration;
    // actual data read in from the roll
    pitchInputGyro = gyroArr[1] - pitchCalibration;
    // actual data read in from the pitch
    yawInputGyro = gyroArr[2] - yawCalibration;
    // actual data read in from the yaw

    // need some actual accurate reading while calculating the roll setpoint

    // rollSetpoint is (500-)
    pitchSetpoint -= pitchAdjust;
    // pitchSetpoint is desired user value - adjusted pitch
    // variables suffixed with 'Adjust' give input in degrees per second,
    // and if pitchSetpoint is a higher dps then that means the drone needs to turn faster.
    rollSetpoint -= rollAdjust;
    // same with roll
    pitchSetpoint = pitchSetpoint/3;
    rollSetpoint = rollSetpoint/3;
    yawSetpoint = yawSetpoint/3;
    /// these three values were scaled down to set bounds of the drone
    // will never go above (500/3)
    // most important lines of code above, we need the right inputs in the pid - in degrees per second
    findPID();
    //pidPitchOutput = 0;
    //pidRollOutput = 0;
    //pidYawOutput = 0;
    // throttle is a continous thing, tilting a part that's not throttle
    // if the drone is not flying, the code should reflect it, and as such, the startingCode should be 1
    if(startingCode && notCalibrating){
      if(throttle >= highestThrottle){
        throttle = highestThrottle;
      }
      // when the drone is stationary need to account for that error...

        //Calculate the pulse for esc 1 (front-right - CCW), PIN 11, esc C
        esc1Value = throttle - pidPitchOutput + pidRollOutput - pidYawOutput;
        //Calculate the pulse for esc 2 (rear-right - CW), PIN 9, esc B
        esc2Value = throttle + pidPitchOutput + pidRollOutput + pidYawOutput;
        //Calculate the pulse for esc 3 (rear-left - CCW), PIN 10, esc A
        esc3Value = throttle + pidPitchOutput - pidRollOutput - pidYawOutput;
        //Calculate the pulse for esc 4 (front-left - CW), PIN 12, esc D
        esc4Value = throttle - pidPitchOutput - pidRollOutput + pidYawOutput;
      /* - low voltage
      if(batteryVoltage <1200 && batteryVoltage >700){
        float comp = (1200-batteryVoltage)/(float(3500));
        esc1Value  = esc1Value * (comp);
      }
      */
      if(esc1Value <= 1000){
        esc1Value = 1000;
      }
      else if(esc1Value >= highestThrottle){
        esc1Value = highestThrottle;
      }

      if(esc2Value <= 1000){
        esc2Value = 1000;
      }
      else if(esc2Value >= highestThrottle){
        esc2Value = highestThrottle;
      }

      if(esc3Value <= 1000){
        esc3Value = 1000;
      }
      else if(esc3Value >= highestThrottle){
        esc3Value = highestThrottle;
      }

      if(esc4Value <= 1000){
        esc4Value = 1000;
      }
      else if(esc4Value >= highestThrottle){
        esc4Value = highestThrottle;
      }

    }
    else if (notCalibrating && !(startingCode)){
      // case where startingCode is 0 - the drone isn't in starting mode yet
      esc1Value = 1000;
      esc2Value = 1000;
      esc3Value = 1000;
      esc4Value = 1000;
    }
    // writing the necessary pulse to each individual ESC based on the pid controller algorithm; each value is different
    // the refresh rate of these escs is 50-60 Hz - take the lower end of it and have a loop that runs every 20,000us because that
    // is the max interval to send pulse to escs
    if(notCalibrating && startingCode){
      // when the escs are being calibrated, we don't want to execute this line of code.
      // otherwise write the desired throttle output to the escs
      esc1.writeMicroseconds(esc1Value);
      esc2.writeMicroseconds(esc2Value);
      esc3.writeMicroseconds(esc3Value);
      esc4.writeMicroseconds(esc4Value);
    }
    //delay(200); // just a simple delay, will need microsecond timer later
    while(micros() - timerDrone <= del);
    timerDrone = micros();
    // reset the timer for the next cycle of sending a command to the drone

    // first read the data and take note of the angle
    readData();

    // adding the cumulative roll angle as the integral of deg/s.

    rollAngle += (rollInputGyro * timerVal);
    pitchAngle += (pitchInputGyro * timerVal);
    yawAngle += (yawInputGyro* timerVal);
    // pi arctan(1)/4;ss

    // a check to see if the yaw has transferred to the roll and/or pitch angle
    // subtract or add the angle if necessary based on these factors
    rollAngle += pitchAngle * sin(gyroArr[2] * timerVal * (Pi / 180));
    pitchAngle -= rollAngle * sin(gyroArr[2] * timerVal * (Pi / 180));

    float accelerationNetVector = accArr[3];
    float anglePitchAcceleration = 0;
    float angleRollAcceleration = 0;
    if(abs(accArr[0]) < accelerationNetVector){
      anglePitchAcceleration = asin(float(accArr[1])/accelerationNetVector) * 57.296;
    }

    if(abs(accArr[1]) < accelerationNetVector){
      angleRollAcceleration = asin(float(accArr[0])/accelerationNetVector) * -57.296;
    }

    pitchAngle = (pitchAngle * 0.994) + (anglePitchAcceleration * 0.006);
    rollAngle = (rollAngle * 0.994) + (angleRollAcceleration * 0.006);
    // a way to prevent the extra noise from the vibrations of the escs
    // will subtract from the pitch and roll setpoint
    // the bigger the angle, the harder the drone will fight back to get back to position
    pitchAdjust = pitchAngle * 15;
    rollAdjust = rollAngle * 15;
    // both need to be in terms of the angle of the pitch and roll
    // how much we need to adjust by
    if(!autoLeveling){
      pitchAdjust = 0;
      rollAdjust = 0;
    }
    // pitch and roll correction
    //(3.142(PI) / 180degr) The Arduino sin function is in radian
    // find integral of dps for angular position of drone
    // reset microsecond timer
  /*
    batteryVoltage = batteryVoltage * 0.92 + (analogRead(0) + 65) * 0.09853;
    if(batteryVoltage <1200 && batteryVoltage >700){
      float comp = (1200-batteryVoltage)/(float(3500));
      esc1Value  = esc1Value * (comp);
    }
*/
}
void gyroCalibrations(){
  rollCalibration = 0;
  pitchCalibration = 0;
  yawCalibration = 0;
  for(int i = 0; i< 2000; i++){
    rollCalibration = rollCalibration + gyroArr[0];
    pitchCalibration = pitchCalibration + gyroArr[1];
    yawCalibration = yawCalibration + gyroArr[2];
    readData();
    delay(1);
    // delay 1  millisecond just to indicate to the user that is takes time
    // adding up all of the totals so that we can get the average
    // read the data and put the information into an array that is basically global to the whole program....
  }
  rollCalibration = (rollCalibration/2000);
  pitchCalibration = (pitchCalibration/2000);
  yawCalibration = (yawCalibration/2000);
  // finds the calibration when the gyro is completely still
}


void calibrateESC(){
  sendUnison(MAX_PULSE_LENGTH);
  //Serial.print("Preparing to send min pulse for arming sequence... Plug in the battery now.");
  // later ... have something that automatically handles this
  delay(4000);
  sendUnison(MIN_PULSE_LENGTH);
  delay(3000);
  //Serial.print("Hopefully all four escs should be calibrated... and made the successful startup sound.");
  startingCode = 1;
}
// a simple testing function to go from the minimum to maximum pulse length
void test()
{
  // occurs after the drone's escs are calibrated.
  // pulse of 1000 to 1400,intervals of 10
    for (int i = MIN_PULSE_LENGTH; i <= 1400; i += 10) {
        //Serial.print(i);
        //Serial.print('\n');
        sendUnison(i);
        delay(200);
    }
    //Serial.println("STOP");
    // going back to writing minimum pulse length
    sendUnison(MIN_PULSE_LENGTH);
}

void readData(){
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  // getthing even of each sensor, and each one will have it's own value
  //Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  //Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  //Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");
  float accX = a.acceleration.x * a.acceleration.x;
  float accY = a.acceleration.y * a.acceleration.y;
  float accZ = a.acceleration.z * a.acceleration.z;
  float xAndy = sqrtf(accX + accY);
  float xyA = xAndy * xAndy;
  float accelerationResult = sqrtf(xyA + accZ);
  accArr[0] = a.acceleration.x;
  accArr[1] = a.acceleration.y;
  accArr[2] = a.acceleration.z;
  accArr[3] = accelerationResult;
  //Serial.print("Magnetic field X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  //Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  //Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");
  // degrees per second
  float gyroX = g.gyro.x * g.gyro.x;
  float gyroY = g.gyro.y * g.gyro.y;
  float gyroZ = g.gyro.z * g.gyro.z;
  float xY = sqrtf(gyroX+gyroY);
  float xYNet = xY * xY;
  float gyroResult = sqrtf(xYNet + gyroZ);
  gyroArr[0] = g.gyro.x;
  gyroArr[1] = g.gyro.y;
  gyroArr[2] = g.gyro.z;
  gyroArr[3] = gyroResult;
// naive filter commented out
  /*
  */
}
// kalman filter to filter out noise - do we need it for now?



void routine(){
  // to write: will write a straightforward function that performs a rudimentary
  // flight routine where once this function is started, the drone will fly up a little bit and then down
 // will later not be needed, for testing purposes currently.

 // start Drone, calibrate everything

 // begin hovering for s seconds at altitude a

 // lower back down to the ground after this.

}

unsigned int calcTimeSpan(int hz){
  return ((1/hz) * 1000000);
}
