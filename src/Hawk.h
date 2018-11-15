#ifndef Hawk_h
#define Hawk_h

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
#define MIN_PULSE_LENGTH 1000
// Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000
// Maximum pulse length in µs
// in order to have stabilize mode, need to keep track of the current angle of the drone

class Hawk{
public:
  Hawk(int hertz, int minPulse, int maxPulse); // =
  // drone constructor
  void setPIDParameters(float p, float i, float d, int maxer, int axis); // =
  // allows user to set the pid parameters of the drone for the axes of rotation

  void beginProcess(); // =
  // start the initial setup of the drone as well as the beginning of the process

  void sendAll(int escValue); // =
  // sends a uniform signal to all of the escs

  void calcPID(); // =
  // calculates the pid given the drone's current state.

  void readData(); // =
  // reads the data, will need to apply a kalman filter here

  void calibrateESCS(); // =
  /* calibration of the escs that isn't currently required yet as the drone is
  not yet autonomous
  */

  void display(); // =
  // displays drone's instructions in case the user wants to see it

  void userInputHandler();
  // handles the input given the key the user pressed

  void edgesCheck(); // =
  // checks to make sure the throttle doesn't go out of bounds

  void setPoints(); // =
  /* ensures that the desired axis setpoints for the user are all in bounds
  before the pid algorithm actually runs.
  */

  int throttleEval(); // =
  // post pid algorithm throttle check

  void angleAdjust();
  // adjusts the angle based on the self stabilizing function, near end of loop

  void gyroCalibrate(); // =
  // mandatory calibrating of the drone to eliminate any possible gyro offset

  void extendedKalman(); // =, barebones code written, implementation not done yet
  // a kalman filter to get the most accurate potrayal of the data
  unsigned int calcTimeSpan(int hz); // =
  // calculates the time in between sending a signal to all of the escs.
  void perform(); // =
  // without this, no signals are sent to escs

private:

  Servo esc1, esc2, esc3, esc4;

  unsigned int span;
  // servos for the hawk

  const float timerVal;
  /* time in between each successive iteration of the loop,
  goal is to integrate degrees/sec to determine drone's angular position.

  */

  Adafruit_LSM9DS1 priSensor;
  // sensor!

  float pi;
  // PI!
  int highestPulse, lowestPulse;

  int absoluteHigh, absoluteLow;
  // lowest and highest pulse to be sent by this flight controller


  int accelerationX, accelerationY, accelerationZ, accelerationNet;
  int throttle;

  // below are all internal variables of the drone to keep track of its state
  float throttleValue, pitchValue, yawValue, rollValue;
  //

  float PgainRoll, IgainRoll, DgainRoll;
  int pidMaxRoll;

  float PgainPitch, IgainPitch, DgainPitch;
  int pidMaxPitch;

  float PgainYaw, IgainYaw, DgainYaw;
  int pidMaxYaw;


  float angleRollAcc, anglePitchAcc, anglePitch, angleRoll;

  int rollAngle, pitchAngle, yawAngle;

  float voltage, pidErrorTemp;
  float iMemRoll, iMemPitch, iMemYaw;
  float rollSetpoint, pitchSetpoint, yawSetpoint;
  float rollInputGyro, pitchInputGyro, yawInputGyro;
  float pidRollOutput, pidPitchOutput, pidYawOutput;
  float pidLastRoll_D_Error, pidLastPitch_D_Error, pidLastYaw_D_Error;
  int esc1Value, esc2Value, esc3Value, esc4Value;
  // what the user types in
  char userInput;
  unsigned long timerDrone;
  // microsecond timer based on the drone's pulse regarding the escs.
  // arrays for the roll, pitch and yaw for the keybpard
  float rollCalibration, pitchCalibration, yawCalibration;
  int startingCode;
  float rollAdjust, pitchAdjust;
  bool autoLeveling, notCalibrating;

  // arrays to hold the data from the sensors of the drone at any time.
  float gyroArr[4];
  float accArr[4];

  float minimizeAbsValue(float previous, float toChange);

  // keep the sensor private!
  // calculates the time span for the drone
  // private:
  /*
    4 escs

  */

};



#endif
