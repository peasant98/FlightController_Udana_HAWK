#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include "..\resources\Adafruit_Sensor.h"
#include "..\resources\Adafruit_LSM9DS1.h"
#include "..\resources\Adafruit_LSM9DS1.cpp"
#include <math.h>
#include "Hawk.h"


Hawk::Hawk(int hertz, int minPulse, int maxPulse){
  priSensor = Adafruit_LSM9DS1();
  // sensor!
  pi = 3.14159265358979323846
  gyroArr = {0,0,0,0};
  accArr = {0,0,0,0};
  autoLeveling = true;
  span = calcTimeSpan(hertz);
  highestPulse = maxPulse;
  lowestPulse = minPulse;
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
  if(!priSensor.begin())
{
  // issue wit h connecting
  Serial.println("Ooops, no LSM9DS1 detected ... Check your wiring!");
}
else{
  // setting up acceleration, magnetism and gyro
  priSensor.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  // 2.) Set the magnetometer sensitivity
  priSensor.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
  // max dps is 24
  priSensor.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
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
  rollCalibration = 0;
  pitchCalibration = 0;
  yawCalibration = 0;


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




}
