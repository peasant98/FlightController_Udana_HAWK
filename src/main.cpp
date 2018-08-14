#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <Servo.h>

#include "..\resources\Adafruit_Sensor.h"
#include "..\resources\Adafruit_LSM9DS1.h"
#include "..\resources\Adafruit_LSM9DS1.cpp"
#include <math.h>


// written by Matthew Strong, sophomore at CU Boulder


// I will create the PID controller and algorithm for handling the necessary aspects here
// and add the code here (the Adafruit LSM9DSL) is already working perfectly here

// including all of the necessary libraries
//#include "..\resources\Adafruit_Sensor.h"
//#include "..\resources\Adafruit_LSM9DS1.h"



#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000

#define LED3 8
#define LED2 7
#define LED1 6
#define LED0 5

// these are arbitrary values that will be changed later


// p, i, and d settings for the roll
float PgainRoll = 1.4;
float IgainRoll = 0.04;
float DgainRoll = 18.0;
// maximum output of the controller
int pidMaxRoll = 400;

// p, i, and d settings for the pitch
float PgainPitch = PgainRoll;
float IgainPitch = IgainRoll;
float DgainPitch = DgainRoll;
int pidMaxPitch = pidMaxRoll;

// p, i, and d settings for the yaw
float PgainYaw = 4.0;
float IgainYaw = 0.02;
float DgainYaw = 0.0;

int pidMaxYaw = pidMaxRoll;


// boolean leveling = true;
int accelerationX, accelerationY, accelerationZ, accelerationNet;
int throttle, batteryVoltage;

float pidErrorTemp;
float iMemRoll, iMemPitch, iMemYaw;
float rollSetpoint, pitchSetpoint, yawSetpoint;
float rollInputGyro, pitchInputGyro, yawInputGyro;
float pidRollOutput, pidPitchOutput, pidYawOutput;
float pidLastRoll_D_Error, pidLastPitch_D_Error, pidLastYaw_D_Error;

Servo esc1, esc2, esc3, esc4;

char data;


Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// define any necessary pins here

// declaring functions in advance
void test();
void displayInstructions();
//Motor 1 : front left - clockwise
//Motor 2 : front right - counter-clockwise
//Motor 3 : rear left - clockwise
//Motor 4 : rear left - counter-clockwise
// each one connected as an esc using the Servo library


// here I will declare all of the 'global' variables that I will be able to use in the setup loop,
// the loop loop, and any other supporting functions that I need

void setup() {
    // put your setup code here, to run once:
    // the main code that is very similar to the arduino code
    Serial.begin(9600);
    // the primary sensor
    Serial.println("Udana HAWK Firing Up....");
    // seeing if the sensor works and is connected correctly
    if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS1 ... check your connections */
    Serial.println("Ooops, no LSM9DS1 detected ... Check your wiring!");
    while(1);
  }
  else{
    Serial.println("Success!");
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

    // 3.) Setup the gyroscope
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
    //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
    //pinMode(LED3, OUTPUT);
    //pinMode(LED2, OUTPUT);
    //pinMode(LED1, OUTPUT);
    //pinMode(LED0, OUTPUT);

    // need to calibrate the gyros when the drone is stationary
    for(int i = 0; i< 2000; i++){

    }



    esc1.attach(10, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    esc2.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    esc3.attach(12, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    esc4.attach(11, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

    Serial.println("Calbrations...");
    displayInstructions();
    // setting up all of the lights based on the direction that is selected by the user
    // pin mode for a led light to caclulate acceleration
  }



}
void displayInstructions()
{
    Serial.println("Choose an Option:");
    Serial.println("\t0 : Send min throttle");
    Serial.println("\t1 : Send max throttle");
    Serial.println("\t2 : Run test function\n");
}


void findPID(){
  // ahh. The most important of them all.


}

void loop() {
  //int x = 0;
  //int x = 1;
  //digitalWrite(LED, HIGH)

  if (Serial.available()) {
        data = Serial.read();

        switch (data) {
            // 0
            case 48 : Serial.println("Sending minimum throttle");
                      esc1.writeMicroseconds(MIN_PULSE_LENGTH);
                      esc2.writeMicroseconds(MIN_PULSE_LENGTH);
                      esc3.writeMicroseconds(MIN_PULSE_LENGTH);
                      esc4.writeMicroseconds(MIN_PULSE_LENGTH);
            break;

            // 1
            case 49 : Serial.println("Sending maximum throttle");
                      esc1.writeMicroseconds(MAX_PULSE_LENGTH);
                      esc2.writeMicroseconds(MAX_PULSE_LENGTH);
                      esc3.writeMicroseconds(MAX_PULSE_LENGTH);
                      esc4.writeMicroseconds(MAX_PULSE_LENGTH);
            break;

            // 2
            case 50 : Serial.print("Running test in 3");
                      delay(1000);
                      Serial.print(" 2");
                      delay(1000);
                      Serial.println(" 1...");
                      delay(1000);
                      test();
            break;
        }
    }
  /*
  while(Serial.available()>0){
    // while we can read from Serial
    int mystring = Serial.read();
    String varx = "";
    varx = varx + char(mystring);
    //Serial.println(char(mystring));
    int myInter = varx.toInt();
    if(myInter == 1){
      digitalWrite(LED3, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED1, LOW);
      digitalWrite(LED0, LOW);
    }
    else{
      digitalWrite(myInter, HIGH);
    }


    // lights up light
    // reads int for light that is chosen

  }
  */


  /*




  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);
  // getthing even of each sensor, and each one will have it's own values


  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");
  float accX = a.acceleration.x * a.acceleration.x;
  float accY = a.acceleration.y * a.acceleration.y;
  float accZ = a.acceleration.z * a.acceleration.z;
  float xAndy = sqrtf(accX + accY);
  float xyA = xAndy * xAndy;
  float result = sqrtf(xyA + accZ);
  // creating total 3-d acceleration vector that finds net acceleration for the sensor, should be 9.8 to 9.9 at rest noramlly

  Serial.print("The 3-d vector for acceleration is: "); Serial.print(result); Serial.println("m/s^2 ");
  if(result>11){
    Serial.print("Slow down please!"  );
    digitalWrite(LED3, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED0, HIGH);

  }
  else{
    digitalWrite(LED3, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED1, LOW);
    digitalWrite(LED0, LOW);
  }
  // printing out all of the values
  Serial.print("Magnetic field X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");

  Serial.print("Roll: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  Serial.print("Pitch: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  Serial.print("Yaw: "); Serial.print(g.gyro.z);      Serial.println(" dps");

  Serial.println();
  Serial.println();
  */


  // calculate the acceleration, magnetism, and gyro every second

  // maybe use this delay for later....
  //delay(1000);
    // put your main code here, to run repeatedly:
}


void calibrateESC(){
  esc1.writeMicroseconds(MAX_PULSE_LENGTH);
  esc2.writeMicroseconds(MAX_PULSE_LENGTH);
  esc3.writeMicroseconds(MAX_PULSE_LENGTH);
  esc4.writeMicroseconds(MAX_PULSE_LENGTH);
  delay(5000);
  // wait 5 seconds before anything can happen

  Serial.print("Preparing to send min pulse for arming sequence...");

  esc1.writeMicroseconds(MIN_PULSE_LENGTH);
  esc2.writeMicroseconds(MIN_PULSE_LENGTH);
  esc3.writeMicroseconds(MIN_PULSE_LENGTH);
  esc4.writeMicroseconds(MIN_PULSE_LENGTH);
  delay(1000);
  Serial.print("Hopefully all four escs should be calibrated...");
  // 1 second delay should enable the esc to know where the max and min pulse are each at....

}



void test()
{
    for (int i = MIN_PULSE_LENGTH; i <= MAX_PULSE_LENGTH; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);

        esc1.writeMicroseconds(i);
        esc2.writeMicroseconds(i);
        esc3.writeMicroseconds(i);
        esc4.writeMicroseconds(i);

        delay(200);
    }

    Serial.println("STOP");
    // going back to writing minimum pulse length
    esc1.writeMicroseconds(MIN_PULSE_LENGTH);
    esc2.writeMicroseconds(MIN_PULSE_LENGTH);
    esc3.writeMicroseconds(MIN_PULSE_LENGTH);
    esc4.writeMicroseconds(MIN_PULSE_LENGTH);
}



void readData(){
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);
  // getthing even of each sensor, and each one will have it's own values


  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");
  float accX = a.acceleration.x * a.acceleration.x;
  float accY = a.acceleration.y * a.acceleration.y;
  float accZ = a.acceleration.z * a.acceleration.z;
  float xAndy = sqrtf(accX + accY);
  float xyA = xAndy * xAndy;
  float result = sqrtf(xyA + accZ);
  // creating total 3-d acceleration vector that finds net acceleration for the sensor, should be 9.8 to 9.9 at rest noramlly

  Serial.print("The 3-d vector for acceleration is: "); Serial.print(result); Serial.println("m/s^2 ");

  // printing out all of the values
  Serial.print("Magnetic field X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");

  Serial.print("Roll: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  Serial.print("Pitch: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  Serial.print("Yaw: "); Serial.print(g.gyro.z);      Serial.println(" dps");

  Serial.println();
  Serial.println();



  // calculate the acceleration, magnetism, and gyro every second

  // maybe use this delay for later....
  //delay(1000);
    // put your main code here, to run repeatedly:






}
