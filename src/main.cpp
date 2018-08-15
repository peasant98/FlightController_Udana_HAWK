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

float gyroArr[4], accelerationArr[4];
float rollCalibration, pitchCalibration, yawCalibration;

int startingCode;
// some arrays to keep track of gyro and acceleration so far

// define any necessary pins here

// declaring functions in advance
void test();
void displayInstructions();
void readData();
void calibrateESC();
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
    startingCode = 0;
    // need to calibrate the gyros when the drone is stationary


    // must be stationary or the drone will not work!!!
    for(int i = 0; i< 2000; i++){
      readData();
      rollCalibration = rollCalibration + gyroArr[0];
      pitchCalibration = pitchCalibration + gyroArr[1];
      yawCalibration = yawCalibration + gyroArr[2];
      delay(4);
      // delay 4 milliseconds
      // adding up all of the totals so that we can get the average
      // read the data and put the infomration into an array that is basically global to the whole program....

    }
    rollCalibration = rollCalibration/2000;
    pitchCalibration = pitchCalibration/2000;
    yawCalibration = yawCalibration/2000;
    // subtract that everything once we read in dps input for the gyro
    // subtract this from the current reading for the best relative value
    // will account for this as the drone is moving
    // getting the averages of everything thus far




    esc1.attach(10, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    esc2.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    esc3.attach(12, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    esc4.attach(11, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

    Serial.println("Calbrations...");

    // the question is, when will we start this


    // now for calbrating and arming the escs (one still doesn't work..)
    calibrateESC();
    // after this and the user has plugged in the drone we will be ready to begin
    // startingCode reset to account for this

    displayInstructions();
    // all steps before doing anything
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

// pid algorithm is here, once we have the desired user inputs we can do stuff
void findPID(){

  pidErrorTemp = rollInputGyro - rollSetpoint;
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
  // but alas, we are not!

  pidErrorTemp = pitchInputGyro - pitchSetpoint;
  iMemPitch += IgainPitch * pidErrorTemp;
  // the intergral pitch is the gain multiplied by the error so far

  if(iMemPitch > pidMaxPitch){
    iMemPitch = pidMaxPitch;
  }
  else if(iMemPitch < (pidMaxPitch*-1)){
    iMemPitch = (pidMaxPitch * -1);
  }
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
  pidYawOutput = PgainYaw * pidErrorTemp + iMemPitch * (pidErrorTemp - pidLastYaw_D_Error);
  pidLastYaw_D_Error = pidErrorTemp;
  // ahh. The most important of them all.
  // the most import outputs are :
  // pidYawOutput
  // pidRollOutput
  // pidPitchOutput

}

void loop() {
  //int x = 0;
  //int x = 1;
  //digitalWrite(LED, HIGH)
  // printing some kind of instructions here

  // steps
  // get the data
  // convert it to degrees
  // use the pid controller to figure out the outputs
  //calibrateESC();
  // do stuff with the motors for the final answer

  Serial.print("The value of the starting code here should be 1.");

  int keyPress;

  // hypothetical example where is key is 11 then the drone will shut down.
  if(keyPress == 11 and startingCode == 1){
    esc1.writeMicroseconds(MIN_PULSE_LENGTH);
    esc2.writeMicroseconds(MIN_PULSE_LENGTH);
    esc3.writeMicroseconds(MIN_PULSE_LENGTH);
    esc4.writeMicroseconds(MIN_PULSE_LENGTH);
    Serial.print("Bringing down the values down to 0.");
    startingCode = 0;
  }
  if(keyPress == 10 and startingCode == 0){
      // example where the motor will be started again
      calibrateESC();
      iMemRoll = 0;
      iMemPitch = 0;
      iMemYaw = 0;
      pidLastRoll_D_Error = 0;
      pidLastPitch_D_Error = 0;
      pidLastYaw_D_Error = 0;
      // resetting all of the pid values when everything is restarted.
      Serial.print("Restarting the HAWK.");
      startingCode = 1;
  }


  // the pid setpoints are determined by the receiver inputs, and are in degrees per second.

  



  readData();



  // will change the values in the gyroArr loop

  // all of the setpoints are determined by the input from the receiver, in this case, the key press on the keyboard.
  rollSetpoint  = 0;
  pitchSetpoint = 0;
  yawSetpoint = 0;
  rollInputGyro = gyroArr[0];
  pitchInputGyro = gyroArr[1];
  yawInputGyro = gyroArr[2];


  // possibly have some kind of calibration for the angles HERE

  // need to calculate for angles here

  // all of the inputs based on reading the data each time



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
  Serial.print("Preparing to send min pulse for arming sequence... Plug in the battery now.");
  delay(4000);
  // wait 5 seconds before anything can happen


  esc1.writeMicroseconds(MIN_PULSE_LENGTH);
  esc2.writeMicroseconds(MIN_PULSE_LENGTH);
  esc3.writeMicroseconds(MIN_PULSE_LENGTH);
  esc4.writeMicroseconds(MIN_PULSE_LENGTH);
  delay(1000);
  Serial.print("Hopefully all four escs should be calibrated...");
  startingCode = 1;
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
  float accelerationResult = sqrtf(xyA + accZ);
  accelerationArr[0] = a.acceleration.x;
  accelerationArr[1] = a.acceleration.y;
  accelerationArr[2] = a.acceleration.z;
  accelerationArr[3] = accelerationResult;

  // creating total 3-d acceleration vector that finds net acceleration for the sensor, should be 9.8 to 9.9 at rest noramlly

  Serial.print("The 3-d vector for acceleration is: "); Serial.print(accelerationResult); Serial.println("m/s^2 ");

  // printing out all of the values
  Serial.print("Magnetic field X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");



  Serial.print("Roll: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  Serial.print("Pitch: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  Serial.print("Yaw: "); Serial.print(g.gyro.z);      Serial.println(" dps");
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
  Serial.println();
  Serial.println();
  // just printing some extra lines here



  // calculate the acceleration, magnetism, and gyro every second

  // maybe use this delay for later....
  //delay(1000);
    // put your main code here, to run repeatedly:






}
