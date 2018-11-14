#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include "..\resources\Adafruit_Sensor.h"
#include "..\resources\Adafruit_LSM9DS1.h"
#include "..\resources\Adafruit_LSM9DS1.cpp"
#include <math.h>
#include "Hawk.h"


Hawk::Hawk(unsigned int hertz, int minPulse, int maxPulse){
  

}

unsigned int Hawk::calcTimeSpan(int hz){
  return ((1/hz) * 1000000);
}




// c++ implementation of the HAWK's inner code.
