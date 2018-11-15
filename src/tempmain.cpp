#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include "..\resources\Adafruit_Sensor.h"
#include "..\resources\Adafruit_LSM9DS1.h"
#include "..\resources\Adafruit_LSM9DS1.cpp"
#include <math.h>
#include "Hawk.h"
// including the default Udana HAWK class to work with


// written by Matthew Strong
// Code for the Udana HAWK Drone

// first 'version' is based on a drone that can be controlled by a keyboard
#define Pi 3.1415926589793238
#define MIN_PULSE_LENGTH 1000
// Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000


Hawk udanaHawk(50, 1000, 1950);
// udana hawk that's global to the whole program.


// setup
void setup(){

  Serial.begin(9600);
  //set pid parameters
  udanaHawk.setPIDParameters(1.2, 0.04, 3.5, 400, 0);
  udanaHawk.setPIDParameters(1.2, 0.04, 3.5, 400, 1);
  udanaHawk.setPIDParameters(4.0, 0.02, 0.0, 400, 2);
  // setting the pid parameters for the drone
  udanaHawk.beginProcess();
  /* begin the process of calibrating the gyro,
  connecting all of the necessary escs, so that
  the user is ready to interact with the drone.

  */

}


// loop
void loop(){
  udanaHawk.userInputHandler();
  // handler for user input
  udanaHawk.edgesCheck();
  // ensures that numbers don't go too high or too low
  udanaHawk.setPoints();
  // checks the setpoints of the axes of the drone
  udanaHawk.calcPID();
  // pid algorithm to interpret desired vs measured data
  udanaHawk.throttleEval();
  /* ensures that the resulting throttle to the drone
  isn't too high or too low for each esc
  */
  udanaHawk.perform();
  // if notcalibrating and started, send the signals!
  udanaHawk.angleAdjust();
  /* after the remaining microsecond timer has elapsed,
  read the data again (will be applying Extended Kalman Filter later)
  and set the angle adjust to what is needed to stabilize the drone.
  */



}
