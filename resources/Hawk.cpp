#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <math.h>
#include "Hawk.h"

// next steps: implement a better way to control the
// desired pitch, yaw, and roll of the user
// by simulating the receiver, transmitter better.


Hawk::Hawk(int hertz, int minPulse, int maxPulse, bool selfStabilizing){
  priSensor = Adafruit_LSM9DS1();
  // sensor!
  pi = 3.14159265358979323846;
  autoLeveling = selfStabilizing;
  span = calcTimeSpan(hertz);
  highestPulse = maxPulse;
  lowestPulse = minPulse;
  absoluteHigh = 2000;
  absoluteLow = 1000;
  timerVal = 1.0/hertz;
  //  set arrays equal to = {0,0,0,0}
  // set autoLeveling to true.

}

unsigned int Hawk::calcTimeSpan(int hz){
  return ((1/hz) * 1000000);
}
// setting the pid parameters for the drone to later be used in pid algorithm
void Hawk::setPIDParameters(float p, float i, float d, int maxer, int axis){
  // set the pid based on the axis, etc
  if (axis==0){
    PgainRoll = p;
    IgainRoll = i;
    DgainRoll = d;
    pidMaxRoll = maxer;
    // roll
  }
  else if (axis==1){
    PgainPitch = p;
    IgainPitch = i;
    DgainPitch = d;
    pidMaxPitch = maxer;
    // pitch
  }
  else if (axis==2){
    PgainYaw = p;
    IgainYaw = i;
    DgainYaw = d;
    pidMaxYaw = maxer;
    // yaw
  }
  // all other numbers can be disregarded.
}

void Hawk::beginProcess(){
  // begin the full process that happens in the setup loop here
  // set ups the sensors and the limits of these sensors.
  if(!priSensor.begin())
{
  // issue wit h connecting
  Serial.println("Ooops, no LSM9DS1 detected ... Check your wiring!");
}
else{
  // setting up acceleration, magnetism and gyro
  priSensor.setupAccel(priSensor.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  // 2.) Set the magnetometer sensitivity
  priSensor.setupMag(priSensor.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
  // max dps is 24
  priSensor.setupGyro(priSensor.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LS  M9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
  startingCode = 0;
  // initial values
  throttleValue = 1000.0;
  pitchValue = 0.0;
  yawValue = 0.0;
  rollValue = 0.0;
  // setting all of the intermediate values to calculate the pitch, roll, yaw, and throttle
  // need to calibrate the gyros when the drone is stationary
  // must be stationary or the drone will not work!!!
  gyroCalibrate();
  // subtract that everything once we read in dps input for the gyro
  // subtract this from the current reading for the best relative value
  // will account for this as the drone is moving
  // getting the averages of everything thus far
  esc1.attach(11, absoluteLow, absoluteHigh);
  esc2.attach(9, absoluteLow, absoluteHigh);
  esc3.attach(10, absoluteLow, absoluteHigh);
  esc4.attach(12, absoluteLow, absoluteHigh);
  // now for calbrating and arming the escs (one still doesn't work..)
  //calibrateESC();
  // requires the user to plug in the battery10
  // will allow the drone to be properly calibrated
  // since the drone has not been started or calibrated at all yet
  //batteryVoltage = (analogRead(0) + 65) * 1.2317;
  rollAngle = 0;
  pitchAngle = 0;
  yawAngle = 0;
  pitchAdjust = 0;
  rollAdjust = 0;


  iMemRoll = 0;
  iMemPitch = 0;
  iMemYaw = 0;
  pidLastRoll_D_Error = 0;
  pidLastPitch_D_Error = 0;
  pidLastYaw_D_Error = 0;


  timerDrone = micros();
  // begin the microsecond timer

}

}

void Hawk::gyroCalibrate(){
  // calibrate the gyro when the drone is currently still.
  rollCalibration = 0;
  pitchCalibration = 0;
  yawCalibration = 0;
  for(int i = 0; i< 2000; i++){
    rollCalibration = rollCalibration + gyroArr[0];
    pitchCalibration = pitchCalibration + gyroArr[1];
    yawCalibration = yawCalibration + gyroArr[2];
    readData();
    // read data modifies gyroArr itself
    delay(1);
    // delay 1  millisecond just to indicate to the user that is takes time
    // adding up all of the totals so that we can get the average
    // read the data and put the information into an array that is basically global to the whole program....
  }
  rollCalibration = rollCalibration/2000;
  pitchCalibration = pitchCalibration/2000;
  yawCalibration = yawCalibration/2000;
  // finds the calibration when the gyro is completely still
}


void Hawk::readData(){
  sensors_event_t a, m, g, temp;
  priSensor.getEvent(&a, &m, &g, &temp);
  float accX = a.acceleration.x * a.acceleration.x;
  float accY = a.acceleration.y * a.acceleration.y;
  float accZ = a.acceleration.z * a.acceleration.z;
  float d2Vec = sqrtf(accX + accY);
  float xyA = d2Vec * d2Vec;
  float accelerationResult = sqrtf(xyA + accZ);
  accArr[0] = a.acceleration.x;
  accArr[1] = a.acceleration.y;
  accArr[2] = a.acceleration.z;
  accArr[3] = accelerationResult;
  // if needed, m.magnetic.x, y, or z is in units of
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
  /* based on the drone's internal state of these arrays, use
  am extended or non-extended kalman filter for the ansewr */
  extendedKalman();
}

// displays all the instructions for the drone
void Hawk::display(){
  Serial.println("Welcome to the Udana HAWK Console.");
  Serial.println("");
  Serial.println("Choose an Option:");
  Serial.println("Q - Increase the Throttle");
  Serial.println("W - Decrease the Throttle");
  Serial.println("A - Increase Pitch");
  Serial.println("S - Decrease Pitch");
  Serial.println("Z - Increase Yaw");
  Serial.println("X - Decrease Yaw");
  Serial.println("E - Increase Roll");
  Serial.println("R - Decrease Roll");
  Serial.println("O - Stop the Drone (Don't do this in midair!)");
  Serial.println("P - Go, get the drone to begin again");
  Serial.println("0 - Send min throttle");
  Serial.println("1 - Send max throttle");
  Serial.println("T - Terminate communication to the drone ");
}

void Hawk::calibrateESCS(){
    sendAll(absoluteHigh);
    //Serial.print("Preparing to send min pulse for arming sequence... Plug in the battery now.");
    // later ... have something that automatically handles this
    delay(4000);
    sendAll(absoluteLow);
    delay(3000);
    startingCode = 1;
}

void Hawk::sendAll(int escValue){
  esc1.writeMicroseconds(escValue);
  esc2.writeMicroseconds(escValue);
  esc3.writeMicroseconds(escValue);
  esc4.writeMicroseconds(escValue);
}

void Hawk::extendedKalman(){
  //code for an extended kalman filter to filter out noise from micro
  // vibrations of the drone
  readData();
  /* first we'll need to read the data before making any
  predictions about the system we are modeling.
  */
}

void Hawk::calcPID(){
  // the pid algorithm for the drone
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
  // changes all of these members of the drone
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

void Hawk::userInputHandler(){
  /* takes user input via serial  connection (to the raspberry pi)
  to convert to something usable
  */
  prevThrottle = throttleValue;
  prevPitch = pitchValue;
  prevYaw = yawValue;
  prevRoll = rollValue;
  notCalibrating = true;
  if (Serial.available()){
    userInput = Serial.read();
    Serial.println(userInput, DEC);
    switch(userInput){

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
          sendAll(absoluteLow);
          startingCode = 0;
        }
        break;

      case 112:
        if(!startingCode){
          calibrateESCS();
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
        sendAll(absoluteLow);
        notCalibrating = false;
        anglePitch = anglePitchAcc;
        angleRoll = angleRollAcc;
        // conduct the testing and starting up the drone

        delay(5000);
        startingCode = 1;
        //startingCode = 1;
        break;

      case 49:
        sendAll(absoluteHigh);
        notCalibrating = false;
        break;

    }
  }
}


float Hawk::minimizeAbsValue(float previous, float toChange){
  /* minimize the absolute value of a value by bringing it closer to
  0. The reason is to simulate a user taking their hands off of the yaw/pitch/
  roll stick, when they are not moving the stick, it automatically
  moves back to the center, in this case, to pitch/yaw/roll value of 0.
  */
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
  /* return the new lowering of this value, simulating the bounce back
  of the drone.

  */
}

void Hawk::edgesCheck(){
  /* make sure the values from userInputHandler()
  are valid and within bounds
  */
  if(notCalibrating and startingCode){
    // the drone must be at the startingCode of 1 to be able to be written a pulse to
    if(prevThrottle == throttleValue){
      throttleValue = prevThrottle;
    }
    if(prevPitch == pitchValue){
      // pitch was not changed, bring it down to the absolute value
      pitchValue  = minimizeAbsValue(prevPitch, pitchValue);
    }
    if(prevYaw == yawValue){
      yawValue = minimizeAbsValue(prevYaw, yawValue);
      // yaw was not changed, bring down to the absolute value
    }
    if(prevRoll == rollValue){
      rollValue = minimizeAbsValue(prevRoll, rollValue);
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
    // integer value needs to be sent to the escs.
    pitchSetpoint = pitchValue;
    yawSetpoint = yawValue;
    rollSetpoint = rollValue;
  }

}

void Hawk::setPoints(){
  /* ensure that the setpoint of each axis is exactly what is desired by the user
  */
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
}

void Hawk::throttleEval(){
  /* evaluates the throttle that needs to be sent to the escs,
  if the throttle is too high, cap it at the max pulse width before
  sending the signal to esc; likewise, if the throttle is too low,
  cap the throttle at the lowest throttle value in order to prevent
  abnormal signals*/
  if(startingCode && notCalibrating){
    if(throttle >= highestPulse){
      throttle = highestPulse;
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
if(batteryVoltage <1200 &&      batteryVoltage >700){
      float comp = (1200-batteryVoltage)/(float(3500));
      esc1Value  = esc1Value * (comp);
    }
    */
    if(esc1Value <= 1000){
      esc1Value = 1000;
    }
    else if(esc1Value >= highestPulse){
      esc1Value = highestPulse;
    }

    if(esc2Value <= 1000){
      esc2Value = 1000;
    }
    else if(esc2Value >= highestPulse){
      esc2Value = highestPulse;
    }

    if(esc3Value <= 1000){
      esc3Value = 1000;
    }
    else if(esc3Value >= highestPulse){
      esc3Value = highestPulse;
    }

    if(esc4Value <= 1000){
      esc4Value = 1000;
    }
    else if(esc4Value >= highestPulse){
      esc4Value = highestPulse;
    }

  }
  else if (notCalibrating && !(startingCode)){
    // case where startingCode is 0 - the drone isn't in starting mode yet
    esc1Value = 1000;
    esc2Value = 1000;
    esc3Value = 1000;
    esc4Value = 1000;
  }

}

void Hawk::perform(){
  if(notCalibrating && startingCode){
    // when the escs are being calibrated, we don't want to execute this line of code.
    // otherwise write the desired throttle output to the escs
    esc1.writeMicroseconds(esc1Value);
    esc2.writeMicroseconds(esc2Value);
    esc3.writeMicroseconds(esc3Value);
    esc4.writeMicroseconds(esc4Value);
  }
}


void Hawk::angleAdjust(){
  while(micros() - timerDrone <= span);
  timerDrone = micros();
  // reset the timer for the next cycle of sending a command to the drone

  // first read the data and take note of the angle
  readData();

  // adding the cumulative roll angle as the integral of deg/s.

  rollAngle += (rollInputGyro * timerVal);
  pitchAngle += (pitchInputGyro * timerVal);
  yawAngle += (yawInputGyro * timerVal);
  // pi arctan(1)/4;ss

  // a check to see if the yaw has transferred to the roll and/or pitch angle
  // subtract or add the angle if necessary based on these factors
  rollAngle += pitchAngle * sin(gyroArr[2] * timerVal * (pi / 180));
  pitchAngle -= rollAngle * sin(gyroArr[2] * timerVal * (pi / 180));

  float accelerationNetVector = accArr[3];
  float anglePitchAcceleration = 0;
  float angleRollAcceleration = 0;
  if(abs(accArr[0]) < accelerationNetVector){
    anglePitchAcceleration = asin(float(accArr[1])/accelerationNetVector) * 57.296;
  }

  if(abs(accArr[1]) < accelerationNetVector){
    angleRollAcceleration = asin(float(accArr[0])/accelerationNetVector) * -57.296;
  }

  pitchAngle = (pitchAngle * 0.95) + (anglePitchAcceleration * 0.05);
  rollAngle = (rollAngle * 0.95) + (angleRollAcceleration * 0.05);
  // a way to prevent the extra noise from the vibrations of the escs
  // will subtract from the pitch and roll setpoint
  // the bigger the angle, the harder the drone will fight back to get back to position
  pitchAdjust = pitchAngle * 15;
  rollAdjust = rollAngle * 15;
  // when the angle is already at 0, the drone does no fighting based on the
  // angle
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


/*

*/
