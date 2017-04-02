// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%% Quadrature Decoder for EMG30 Motors %%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// © UBRobotics 2016
// Written by Ed Chamberlain
// Modified by Tom Brereton
//
// This sketch will read the encoder value for a sinle EMG30 motors
// and print the current count to the serial interface.
//
// Do NOT accidently supply 12V to the Arduino - you will break it.
// You can use the 5V supply for the encoders, remeber to connect
// grounds. Pull encoder signals high using 10K resistors. Internal
// pull-ups are used in this sketch but wil vary by board. Do not
// rely on these for extensive testing.
//
// EMG30 Wiring as follows:
//   Wire colour |  Connection
//   ------------|------------------
//    1 Purple   |  Hall Sensor B Vout
//    2 Blue     |  Hall sensor A Vout
//    3 Green    |  Hall sensor ground
//    4 Brown    |  Hall sensor Vcc
//    5 Red      |  + Motor
//    6 Black    |  - Motor
//
// %%%%%%%%%%%%%%%%%%%%%%%%%%%% About %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// * Read in encoders from both wheels and keep robot straight 
//   via count difference == 0.
// * The Pi will send a packet via a protocol which tells the robot
//   to drive straight, 90 degree left, 90 degree right, backwards.
// * This program contains functions for straight, left, right,
//   backwards.
// * The packet sent from the Pi will call these functions.


// ----------------------------------------------------------------
// ------------------------ Program Plan --------------------------
// ----------------------------------------------------------------

/** 
 * [x] Function to return sensor value of left encoder
 * [x] Function to return sensor value of right encoder
 * [] Function to specify power to master motor (left)
 * [] Function to specify power to slave motor (right)
 * [] Manually tune 'constant of proportionality,' kp
 * [] Calibrate degrees turned using the calibrate function
 * [x] Function to drive a certain distance straight
 * [x] Function to turn left X degrees
 * [x] Function to turn right X degrees
 * [] Function to decode request protocol
 * [] Function to encode response protocol
 */



// ----------------------------------------------------------------
// ----------------------- Libraries ------------------------------
// ----------------------------------------------------------------

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <string>
#include <vector>
#include <string.h>
#include <stdio.h>

// ----------------------------------------------------------------
// ----------------------- Global Variables -----------------------
// ----------------------------------------------------------------

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1 and M2
Adafruit_DCMotor *masterMotor = AFMS.getMotor(1);
Adafruit_DCMotor *slaveMotor = AFMS.getMotor(2);

// pulse counter
// The pulse counter increments by one for every 1/360 rotation (1 degree)
// (CONFIRM last statement)
volatile long masterCount = 0;
volatile long slaveCount = 0;

// Variables for calculating straight distance
const double PIE = 3.1415;
const double WHEEL_DIAMETER_CM = 9.5;
const double CIRCUMFERENCE_CM = PIE * WHEEL_DIAMETER_CM;
const int TICKS_PER_CM = 360 / CIRCUMFERENCE_CM;

// Variables for calculating turning distance
const int CALIBRATER_TICKS = 200;
// degrees rotated must be measured after running the calibration program
// refer to the following website for the experiment
// http://www.robotc.net/wikiarchive/Tutorials/Arduino_Projects/Mobile_Robotics/VEX/Using_encoders_to_make_turns
const int DEGREES_ROTATED = 86;
const int TICKS_PER_10_DEGREES = (CALIBRATER_TICKS * 10) / DEGREES_ROTATED;

/**
 *'Constant of proportionality' which the error is divided by. 
 * Usually this is a number between 1 and 0 the error is multiplied by, 
 * but we cannot use floating point numbers. Basically, it lets us 
 * choose how much the difference in encoder values effects the 
 * final power change to the motor.
 */
int kp = 5;

// This tells the interrupt to run the method on any change
#define intMode CHANGE

#define absolute(X) ((X < 0) ? -1 * X : X)

// ----------------------------------------------------------------
// --------------------------- Setup ------------------------------
// ----------------------------------------------------------------

void setup() {
  // read in from pins 8 & 9 for master wheel
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);

  // read in from pins 10 & 11 for slave wheel
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);

  // init serial
  // We use 250000 for a high speed data transfer
//  Serial.begin(250000);
  Serial.begin(9600);

  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // We attach the encoders for the master wheel (left)
  attachInterrupt(8, masterEncoderA, intMode);
  attachInterrupt(9, masterEncoderB, intMode);

  // We attach the encoders for the slave wheel (right)
  attachInterrupt(10, slaveEncoderA, intMode);
  attachInterrupt(11, slaveEncoderB, intMode);

  AFMS.begin();  // create with the default frequency 1.6KHz
}

// ----------------------------------------------------------------
// --------- Interrupt Service Routines Master Wheel --------------
// ----------------------------------------------------------------

/** 
 * masterEncoderA for the master wheel.
 * If we get a read from pin 7 or 8 but not both, we decrease the 
 * masterCount, otherwise increase.
 */
void masterEncoderA () {
  // decode quadrature
  (digitalRead(8) ^ digitalRead(9)) ? masterCount-- : masterCount++;
  // debug to serial
  Serial.println(masterCount);
}
  
/** 
 * masterEncoderB for the master wheel.
 * If we get a read from pin 7 or 8 but not both, we increase the 
 * masterCount, otherwise decrease.
 */
void masterEncoderB () {
  // decode quadrature
  (digitalRead(8) ^ digitalRead(9)) ? masterCount++ : masterCount--;
  // debug to serial
  Serial.println(masterCount);
}

// ----------------------------------------------------------------
// --------- Interrupt Service Routines Slave Wheel --------------
// ----------------------------------------------------------------

/** 
 * slaveEncoderA for the slave wheel.
 * If we get a read from pin 5 or 6 but not both, we decrease the 
 * slaveCount, otherwise increase.
 */
void slaveEncoderA () {
  // decode quadrature
  (digitalRead(10) ^ digitalRead(11)) ? slaveCount-- : slaveCount++;
  Serial.println(slaveCount);
}
  
/** 
 * slaveEncoderB for the slave wheel.
 * If we get a read from pin 7 or 8 but not both, we increase the 
 * slaveCount, otherwise decrease.
 */
void slaveEncoderB () {
  // decode quadrature
  (digitalRead(10) ^ digitalRead(11)) ? slaveCount++ : slaveCount--;
  Serial.println(slaveCount);
}

// ----------------------------------------------------------------
// -------------------------- Functions ---------------------------
// ----------------------------------------------------------------

/**
 * This function sets the speed of the master motor.
 * A 0 or positive value makes the motor go forward,
 * a negative value makes the motor go backward.
 * Speed ranges between 0(off) to 255(max speed)
 */
void setMasterMotorSpeed(int speed){
  if(speed >= 0){
    masterMotor->run(FORWARD);
    masterMotor->setSpeed(speed);
  } else if (speed < 0){
    masterMotor->run(BACKWARD);
    masterMotor->setSpeed(-1*speed);
  }
//  // turn on motor
//  masterMotor->run(RELEASE);
}


/**
 * This function sets the speed of the slave motor.
 * A 0 or positive value makes the motor go forward,
 * a negative value makes the motor go backward.
 * Speed ranges between 0(off) to 255(max speed)
 */
void setSlaveMotorSpeed(int speed){
  // we take the negative of speed as the slave
  // is facing the opposite direction to the master
  
  if(speed >= 0){
    slaveMotor->run(FORWARD);
    slaveMotor->setSpeed(speed);
  } else if (speed < 0){
    slaveMotor->run(BACKWARD);
    slaveMotor->setSpeed(-1*speed);
  }
//  // turn on motor
//  slaveMotor->run(RELEASE);
}

// ----------------------------------------------------------------
// ----------------------- Main Function --------------------------
// ----------------------------------------------------------------

void loop(){
  setMasterMotorSpeed(100);
  setSlaveMotorSpeed(100);
  
}
