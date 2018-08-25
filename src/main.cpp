#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <Servo.h>

#include "..\resources\Adafruit_Sensor.h"
#include "..\resources\Adafruit_LSM9DS1.h"
#include "..\resources\Adafruit_LSM9DS1.cpp"
#include <math.h>



// written by Matthew Strong, sophomore at CU Boulder
// CTO of Udana Systems, 2018
// Code for the Udana HAWK

// first 'version' is based on a drone that can be controlled by a keyboard, so we can do that
// we have to have some kind of good way to have user keyboard input

// I will create the PID controller and algorithm for handling the necessary aspects here
// and add the code here (the Adafruit LSM9DSL) is already working perfectly here

// issues thus far:
// after calibrating the four escs (in which all four give the desired beep signals), the motors will stop in the middle of going from 1000 to 1500 microseconds
// pulse. The reason is perhaps the battery; but could be related to how fast I given off the signals.... we will seeing

// including all of the necessary libraries
//#include "..\resources\Adafruit_Sensor.h"
//#include "..\resources\Adafruit_LSM9DS1.h"



#define MIN_PULSE_LENGTH 1000
// Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000
// Maximum pulse length in µs

#define LED3 8
#define LED2 7
#define LED1 6
#define LED0 5

// these are arbitrary values that will be changed later

//

// important values for keeping track of things

float throttleValue, pitchValue, yawValue, rollValue;

float angleRollAcc, anglePitchAcc, anglePitch, angleRoll;
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
int throttle;
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

float gyroArr[4], accelerationArr[4];
// arrays for the roll, pitch and yaw for the keybpard
float rollCalibration, pitchCalibration, yawCalibration;

int startingCode;

float rollAdjust, pitchAdjust;

bool autoLeveling = true;
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
    autoLeveling = false;
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
    throttleValue = 1000.0;
    pitchValue = 0.0;
    yawValue = 0.0;
    rollValue = 0.0;
    // setting all of the intermediate values to calculate the pitch, roll, yaw, and throttle
    // need to calibrate the gyros when the drone is stationary
    // must be stationary or the drone will not work!!!

    // setting the calbiration values - might package into a function here
    Serial.println("Calibrating all of the gyro values to zero. Please have the drone stationary.");
    for(int i = 0; i< 2000; i++){
      readData();
      rollCalibration = rollCalibration + gyroArr[0];
      pitchCalibration = pitchCalibration + gyroArr[1];
      yawCalibration = yawCalibration + gyroArr[2];
      delay(3);
      // delay 4 milliseconds
      // adding up all of the totals so that we can get the average
      // read the data and put the infomration into an array that is basically global to the whole program....

    }
    Serial.println("Gyro values calibrated!");
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

    Serial.println("HAWK Beginning....");
    // now for calbrating and arming the escs (one still doesn't work..)
    //calibrateESC();
    // requires the user to plug in the battery

    // will allow the drone to be properly calibrated.


    // after this and the user has plugged in the drone we will be ready to begin
    // startingCode reset to account for this

    displayInstructions();

    startingCode = 0;
    // since the drone has not been started or calibrated at all yet






    batteryVoltage = (analogRead(0) + 65) * 1.2317;





    timerDrone = micros();
    // type of micros is unsigned long here


    // amount in the battery
    // all steps before doing anything
    // setting up all of the lights based on the direction that is selected by the user
    // pin mode for a led light to caclulate acceleration
  }



}
void displayInstructions()
{
    // displays all of the options for the user, only displays once so that it's not displaying over and over and over again.
    Serial.println("Welcome to the Udana HAWK Serial Console!");
    Serial.println("");
    Serial.println("Choose an Option:");
    Serial.println("Q - Increase the Throttle");
    Serial.println("W - Decrease the Throttle");
    // throttle options either bring it down or bring it up....
    Serial.println("A - Up the Pitch");
    Serial.println("S - Down the Pitch");
    Serial.println("Z - Up the Yaw");
    Serial.println("X - Down the Yaw");
    Serial.println("E - Up the Roll");
    Serial.println("R - Down the Roll");
    Serial.println("===============================================");
    Serial.println("O - Stop the Drone (Don't do this in midair)");
    Serial.println("P - Go, get the drone to begin again");
    Serial.println("In the proto-prototype version instructions: ");
    Serial.println("0 : Send min throttle");
    Serial.println("1 : Send max throttle");
    Serial.println("2 : Run test function\n");
}

// pid algorithm is here, once we have the desired user inputs we can calculate what pulse we must send to each of the motors
// the input for the function will be in degrees per second
// note that throttle is independent of these values, that really determines how far up the drone goes, not its x, y, or z orientation and desired
// direction.
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
// the main loop

// start up drone here
  // request a character from the user
  float prevThrottle = throttleValue;
  float prevPitch = pitchValue;
  float prevYaw = yawValue;
  float prevRoll = rollValue;

  bool notCalibrating = true;
  // allow the user the see the options that they can do in order to control the drone
  // serial will be available with the raspberry pi
  if (Serial.available()) {
        // need to modify this on the raspberry pi
        data = Serial.read();
        Serial.print("Received the code: ");
        Serial.println(data, DEC);
        // printing data in decimal repr.
        // the cases here are displayed in order in which the instructions are displayed
        // letters:
        // q, w, a, s, z, x, e, r
        // below are all the ascii codes for the keys that add functionality to the drone
        // roll - x
        // pitch - y
        // yaw - z
        if(data == 113){
          throttleValue+=20.0;
          notCalibrating = true;

        }
        else if(data == 119){
          throttleValue-=20.0;
          notCalibrating = true;

        }
        else if(data == 97){
          pitchValue+=20.0;
          notCalibrating = true;

        }
        else if(data == 115){
          pitchValue-=20.0;
          notCalibrating = true;

        }
        else if(data == 122){
          yawValue+=20.0;
          notCalibrating = true;

        }
        else if(data == 120){
          yawValue-=20.0;
          notCalibrating = true;

        }
        else if(data == 101){
          rollValue+=20.0;
          notCalibrating = true;

        }
        else if(data == 114){
          rollValue-=20.0;
          notCalibrating = true;

        }
        else if(data == 111){
          if(startingCode == 1){
            // if the drone is in the starting condition we can bring it to a stop, not the other way around
            esc1.writeMicroseconds(MIN_PULSE_LENGTH);
            esc2.writeMicroseconds(MIN_PULSE_LENGTH);
            esc3.writeMicroseconds(MIN_PULSE_LENGTH);
            esc4.writeMicroseconds(MIN_PULSE_LENGTH);
            Serial.print("Bringing down the values down to 0.");
            startingCode = 0;
          }
          else{
            Serial.println("You cannot do this based on the starting code.");
          }

        }
        else if(data == 112){
          if(startingCode == 0){
            calibrateESC();
            // recalibrate esc because the starting code is now 0
            iMemRoll = 0;
            iMemPitch = 0;
            iMemYaw = 0;
            pidLastRoll_D_Error = 0;
            pidLastPitch_D_Error = 0;
            pidLastYaw_D_Error = 0;
            // resetting all of the pid values when everything is restarted.
            Serial.print("Restarting the HAWK from being STATIONARY...");
            // the HAWK has already been calibrated at this point in the program though
            // we can begin from here
            startingCode = 1;
          }
          else{
            Serial.println("You cannot do this based on the starting code.");
          }

        }
        else if(data == 48){
          Serial.println("Sending minimum throttle");
          esc1.writeMicroseconds(MIN_PULSE_LENGTH);
          esc2.writeMicroseconds(MIN_PULSE_LENGTH);
          esc3.writeMicroseconds(MIN_PULSE_LENGTH);
          esc4.writeMicroseconds(MIN_PULSE_LENGTH);
          // write minimum throttle
          notCalibrating = false;

        }
        else if(data == 49){
          Serial.println("Sending maximum throttle");
          esc1.writeMicroseconds(MAX_PULSE_LENGTH);
          esc2.writeMicroseconds(MAX_PULSE_LENGTH);
          esc3.writeMicroseconds(MAX_PULSE_LENGTH);
          esc4.writeMicroseconds(MAX_PULSE_LENGTH);
          // write max throttle
          notCalibrating = false;

        }
        else if(data == 50){
          Serial.print("Running test in 3");
          delay(1000);
          Serial.print(" 2");
          delay(1000);
          Serial.println(" 1...");
          delay(1000);
          test();
          // conduct the testing
          notCalibrating = false;
          startingCode = 1;

        }
        // all other cases are just ignored.



        // change up the code here without switch statements


    }
    if(notCalibrating and startingCode == 1){
      // the drone must be at the startingCode of 1 to be able to be written a pulse too
      if(prevThrottle == throttleValue){
        throttleValue = prevThrottle;
          // throttle was not changed
      }
      if(prevPitch == pitchValue){
        // pitch was not changed
        pitchValue = 0;

      }
      if(prevYaw == yawValue){
        yawValue = 0;
        // yaw was not changed

      }
      if(prevRoll == rollValue){
        rollValue = 0;
        // roll was not changed
      }

      // account for edge cases
      // need to get to degrees per second...

      if(throttleValue >= 1950){
        throttleValue = 1950;
      }
      if(throttleValue < (1000)){
        throttleValue = 1000;
      }

      if(pitchValue > 400){
        pitchValue = 400;
      }
      if(pitchValue < -400){
        pitchValue = -400;
      }
      if(rollValue > 400){
        rollValue = 400;
      }
      if(rollValue < -400){
        rollValue = -400;
      }

      if(yawValue > 400){
        yawValue = 400;
      }
      if(yawValue < (-400)){
        yawValue = -400;
      }
      //Serial.println(throttle);
      //Serial.println(pitchValue);
      //Serial.println(rollValue);
      //Serial.println(yawValue);
      // after all of the edge cases, we can set the pidsetpoints to what the user has provided... then calibrate via the angle
      throttle = int(throttleValue);
      pitchSetpoint = pitchValue;
      yawSetpoint = yawValue;
      rollSetpoint = rollValue;

      // here  is simply a test
      esc1.writeMicroseconds(throttle);
      esc2.writeMicroseconds(throttle);
      esc3.writeMicroseconds(throttle);
      esc4.writeMicroseconds(throttle);
      // delay of 200 millseconds - will need to bring down later so that more corrections are made per second
      delay(200);


    }


    // things should be in degrees per seconds


    // reading in the data to get the right inputs
    readData();

    // some code that I had to research that details the angles and whatnot
    // will need for later once the main pid controller works

    anglePitch = anglePitch + (gyroArr[0]* 0.0000611);

    angleRoll = angleRollAcc + (gyroArr[1]*0.0000611);


    anglePitch -= angleRoll * sin(gyroArr[2] * 0.000001066);
    angleRoll += anglePitch * sin(gyroArr[2] * 0.000001066);
    if(abs(accelerationArr[1])< accelerationArr[3]){
      anglePitchAcc = asin((float)accelerationArr[1]/accelerationArr[3])*57.2960;
  }
    if(abs(accelerationArr[0])< accelerationArr[3]){
      angleRollAcc = asin((float)accelerationArr[0]/accelerationArr[3])* (-57.2960);
    }


    anglePitch = anglePitch * 0.9997 + anglePitchAcc * 0.0003;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    angleRoll = angleRoll * 0.9997 + angleRollAcc * 0.0003;

                   //Correct the drift of the gyro roll angle with the accelerometer roll angle.

    pitchAdjust = anglePitch * 15;
    rollAdjust = angleRoll * 15;
    // will have to modify values of these later
    if(!autoLeveling){
      pitchAdjust = 0;
      rollAdjust = 0;
    }

    // correcting for drift
    pitchSetpoint = pitchSetpoint - pitchAdjust;
    rollSetpoint = rollSetpoint - rollAdjust;

    // input is subtracted from what the roll is expected at the stationary surface; that is, when all axes are at 0 ....
    // (0,0,0)
    rollInputGyro = gyroArr[0] - rollCalibration;
    pitchInputGyro = gyroArr[1] - pitchCalibration;
    yawInputGyro = gyroArr[2] - yawCalibration;
    findPID();



    // getting to an actual motor reading HERE
    int highestThrottle = 1950;
    // throttle is a continous thing, tilting a part that's not throttle
    // will affect the pitch/yaw/roll

    // if the drone is not flying, the code should reflect it, and as such, the startingCode should be 1
    if(startingCode == 1){
      if(throttle > highestThrottle){
        // fairly straightforward math right here
        throttle = highestThrottle;
      }
        //Calculate the pulse for esc 1 (front-right - CCW)

        esc1Value = throttle - pidPitchOutput + pidRollOutput - pidYawOutput;
        //Calculate the pulse for esc 2 (rear-right - CW)
        esc2Value = throttle + pidPitchOutput + pidRollOutput + pidYawOutput;

        //Calculate the pulse for esc 3 (rear-left - CCW)
        esc3Value = throttle + pidPitchOutput - pidRollOutput - pidYawOutput;

        //Calculate the pulse for esc 4 (front-left - CW)
        esc4Value = throttle - pidPitchOutput - pidRollOutput + pidYawOutput;
        // based on the direction that the motor needs to spin determines what need to
        // written to each of the escs

      // in the case that the battery voltage is really low
      // either that or everything fails

      if(batteryVoltage <1200 && batteryVoltage >700){
        float comp = (1200-batteryVoltage)/(float(3500));
        esc1Value  = esc1Value * (comp);

        // quite low voltage too - bring pulse down due to voltage drop
      }
      // worrying about the lowest limits here
      if(esc1Value < 1050){
        esc1Value = 1050;
      }
      if(esc2Value < 1050){
        esc2Value = 1050;
      }
      if(esc3Value < 1050){
        esc3Value = 1050;
      }
      if(esc4Value < 1050){
        esc4Value = 1050;
      }
      if(esc1Value >= 2000){
        esc1Value = 2000;
      }
      if(esc2Value >= 2000){
        esc2Value = 2000;
      }
      if(esc3Value >= 2000){
        esc3Value = 2000;
      }
      if(esc4Value >= 2000){
        esc4Value = 2000;
      }
      // the drone would have started HERE

    }
    else{
      // case where startingCode is 0
      esc1Value = 1000;
      esc2Value = 1000;
      esc3Value = 1000;
      esc4Value = 1000;

      // the starting code here is 0, so we can safely assume that the drone is
      // not going to be moving anytime soon....

    }
    // writing the neessary pulse to each individual ESC based on the pid controller algorithm; each value is different
    // due to taking into account direction
    esc1.writeMicroseconds(esc1Value);
    esc2.writeMicroseconds(esc2Value);
    esc3.writeMicroseconds(esc3Value);
    esc4.writeMicroseconds(esc4Value);




// below is all test code that I was playing around with


  /*

  if(!autoLeveling){
    pitchAdjust = 0;
    rollAdjust = 0;
  }
  pitchSetpoint = pitchSetpoint - pitchAdjust;
  rollSetpoint = rollSetpoint - rollAdjust;
  // accounting for the roll and pitch adjustments based on angles
  // we need to look more into this and see if any dividing needs to be done as per the PID controller


  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }
  // already have calculated the net acceleration vector from the answer here


  angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.

  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

  if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
  }



  angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
  angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
  gyro_angles_set = true;

  // need to get pitch, roll, yaw adjust

  // somewhere around here allow the drone to be autoleveling

  // will change the values in the gyroArr loop

  // all of the setpoints are determined by the input from the receiver, in this case, the key press on the keyboard.

  rollInputGyro = gyroArr[0] - rollCalibration;
  pitchInputGyro = gyroArr[1] - pitchCalibration;
  yawInputGyro = gyroArr[2] - yawCalibration;

  findPID();


  // the calculate PID function in all of it's glor....
  // the inputs for this function will be the inputs of the gyro that are read in
  // as well as the setpoints from the hypothetical receivers, in this case....
  // the key press of the keybaard connected to raspberry pi

  batteryVoltage = batteryVoltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  // accounting for the calibration that occurred when the drone was stationary


  // possibly have some kind of calibration for the angles HERE

  // need to calculate for angles here

  // all of the inputs based on reading the data each time
  // account for throttle of the drone...
  int keyPressThrottle;
  int highestThrottle = 1946;
  throttle = keyPressThrottle;
  // throttle is a continous thing, tilting a part that's not throttle
  // will affect the pitch/yaw/roll
  if(startingCode == 1){
    if(throttle > highestThrottle){
      // fairly straightforward math right here
      throttle = highestThrottle;
      //Calculate the pulse for esc 1 (front-right - CCW)
      esc1Value = throttle - pidPitchOutput + pidRollOutput - pidYawOutput;
      //Calculate the pulse for esc 2 (rear-right - CW)
      esc2Value = throttle + pidPitchOutput + pidRollOutput + pidYawOutput;

      //Calculate the pulse for esc 3 (rear-left - CCW)
      esc3Value = throttle + pidPitchOutput - pidRollOutput - pidYawOutput;

      //Calculate the pulse for esc 4 (front-left - CW)
      esc4Value = throttle - pidPitchOutput - pidRollOutput + pidYawOutput;
      // based on the direction that the motor needs to spin determines what need to
      // written to each of the escs


    }
    if(batteryVoltage <1200 && batteryVoltage >700){
      float comp = (1200-batteryVoltage)/(float(3500));
      esc1Value  = esc1Value * (comp);

      // quite low voltage too - bring pulse down due to voltage drop
    }
    // worrying about the lowest limits here
    if(esc1Value < 1050){
      esc1Value = 1050;
    }
    if(esc2Value < 1050){
      esc2Value = 1050;
    }
    if(esc3Value < 1050){
      esc3Value = 1050;
    }
    if(esc4Value < 1050){
      esc4Value = 1050;
    }
    if(esc1Value > 2000){
      esc1Value = 2000;
    }
    if(esc2Value > 2000){
      esc2Value = 2000;
    }
    if(esc3Value > 2000){
      esc3Value = 2000;
    }
    if(esc4Value > 2000){
      esc4Value = 2000;
    }
    // the drone would have started HERE

  }
  else{
    esc1Value = 1000;
    esc2Value = 1000;
    esc3Value = 1000;
    esc4Value = 1000;

    // the starting code here is 1, so we can safely assume that the drone is
    // not going to be moving anytime soon....

  }

  esc1.writeMicroseconds(esc1Value);
  esc2.writeMicroseconds(esc2Value);
  esc3.writeMicroseconds(esc3Value);
  esc4.writeMicroseconds(esc4Value);

*/




// END OF CODE.......



  // next step is to have


  // write the escValues to each respective esc each time that this occurs..... read every 4 microseconds

  // this code here is is to best communication with the pi

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

  */




  // calculate the acceleration, magnetism, and gyro every second
    // currently I do not know the pulse in Hz of my ESC so I am assuming that my 30A ESC is 250 Hz, so every 4000 us we reset the timer.
    //while(micros() - timerDrone < 4000);
    // accounting for the extra time to have the rest of the pulse

    // need to be able to check the pulse of esc in order to do stuff
    // simply having some waiting tiem herr, we will see if we need this later




    //timerDrone = micros();

}


void calibrateESC(){
  esc1.writeMicroseconds(MAX_PULSE_LENGTH);
  esc2.writeMicroseconds(MAX_PULSE_LENGTH);
  esc3.writeMicroseconds(MAX_PULSE_LENGTH);
  esc4.writeMicroseconds(MAX_PULSE_LENGTH);
  Serial.print("Preparing to send min pulse for arming sequence... Plug in the battery now.");
  // later ... have something that automatically handles this
  delay(4000);
  // wait 5 seconds before anything can happen


  esc1.writeMicroseconds(MIN_PULSE_LENGTH);
  esc2.writeMicroseconds(MIN_PULSE_LENGTH);
  esc3.writeMicroseconds(MIN_PULSE_LENGTH);
  esc4.writeMicroseconds(MIN_PULSE_LENGTH);
  delay(4000);
  Serial.print("Hopefully all four escs should be calibrated... and made the successful startup sound.");
  startingCode = 1;
  // 1 second delay should enable the esc to know where the max and min pulse are each at....

}


// a simple testing function to go from the minimum to maximum pulse length
// for the ESCS
void test()
{
  // occurs after the drone's escs are calibrated.
  // pulse of 1000 to 1400
    for (int i = MIN_PULSE_LENGTH; i <= 1400; i += 5) {
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

  //Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  //Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  //Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

  float accX = a.acceleration.x * a.acceleration.x;
  float accY = a.acceleration.y * a.acceleration.y;
  float accZ = a.acceleration.z * a.acceleration.z;
  float xAndy = sqrtf(accX + accY);
  float xyA = xAndy * xAndy;
  float accelerationResult = sqrtf(xyA + accZ);
  // net acceleration vector
  accelerationArr[0] = a.acceleration.x;
  accelerationArr[1] = a.acceleration.y;
  accelerationArr[2] = a.acceleration.z;
  accelerationArr[3] = accelerationResult;

  // creating total 3-d acceleration vector that finds net acceleration for the sensor, should be 9.8 to 9.9 at rest noramlly

  //Serial.print("The 3-d vector for acceleration is: "); Serial.print(accelerationResult); Serial.println("m/s^2 ");

  // printing out all of the values
  //Serial.print("Magnetic field X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  //Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  //Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");


  // degrees per second
  //Serial.print("Roll: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  //Serial.print("Pitch: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  //Serial.print("Yaw: "); Serial.print(g.gyro.z);      Serial.println(" dps");
  float gyroX = g.gyro.x * g.gyro.x;
  float gyroY = g.gyro.y * g.gyro.y;
  float gyroZ = g.gyro.z * g.gyro.z;
  float xY = sqrtf(gyroX+gyroY);
  float xYNet = xY * xY;
  // net gyro std::vector<int> v;
  float gyroResult = sqrtf(xYNet + gyroZ);
  gyroArr[0] = g.gyro.x;
  gyroArr[1] = g.gyro.y;
  gyroArr[2] = g.gyro.z;
  gyroArr[3] = gyroResult;




}
