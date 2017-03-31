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

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // We attach the encoders for the master wheel (left)
  attachInterrupt(8, masterEncoderA, intMode);
  attachInterrupt(9, masterEncoderB, intMode);

  // We attach the encoders for the slave wheel (right)
  attachInterrupt(10, slaveEncoderA, intMode);
  attachInterrupt(11, slaveEncoderB, intMode);
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
}
  
/** 
 * slaveEncoderB for the slave wheel.
 * If we get a read from pin 7 or 8 but not both, we increase the 
 * slaveCount, otherwise decrease.
 */
void slaveEncoderB () {
  // decode quadrature
  (digitalRead(10) ^ digitalRead(11)) ? slaveCount++ : slaveCount--;
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

/**
 * This function determines the total number of ticks needed
 * to travel the milimetres specified to travel. It uses the formula:
 * 
 * Encoder ticks = (360 / circumference) * Distance to travel
 * or
 * Encoder ticks = TICKS_PER_CM * Distance to travel
 */
int straightTickGoalCalculator(int mmToTravel){
  int tickGoal = (TICKS_PER_CM * mmToTravel) / 10;
  return tickGoal;
}

/**
 * This function determines the total number of ticks needed
 * to turn the degrees specified. It uses the formula:
 * 
 * Encoder ticks = TICKS_PER_10_DEGREES * degrees
 */
int turnTickGoalCalculator(int degrees){
  int tickGoal = (TICKS_PER_10_DEGREES * degrees) / 10;
  return tickGoal;
}

/**
 * This function turns the robot 200 encoder ticks and is used to
 * calibrate its turning ability.
 * 
 * (1) The first thing we need to do is place the robot 
 * on a markable surface (e.g. whiteboard, a large sheet of paper, etc.) 
 * and mark a line perpendicular to the robot, straight through 
 * its center.
 * (2) Now, run the program.
 * (3) Next, mark another perpendicular line in the same manner as you 
 * did before, only relative to the robot's new position. 
 * (4) Finally, use a protractor to measure the angle formed between 
 * these two lines, and that will determine the angle through 
 * which the robot turned. 
 * (5) Enter this angle as the value for the variable, DEGREES_ROTATED above.
 */
void calibration(){
  masterCount = 0;
  slaveCount = 0;
 
  // Perform a point turn to the left. We will use lower power 
  // values for more accuracy.
  setMasterMotorSpeed(100);
  setSlaveMotorSpeed(-100);

    // Since the wheels may go at slightly different speeds 
    // due to manufacturing tolerances, etc., we need to test 
    // both encoders and control both motors separately. This 
    // may result in one motor going for longer than another but 
    // it will ultimately result in a much more accurate turn.

    while(slaveCount < 200 || masterCount > -200){
      
      if(slaveCount > 200){
        setSlaveMotorSpeed(0);
      }
      
      if(masterCount < -200){
        setMasterMotorSpeed(0);
      }
    }
}

/**
 * This fuctions rotates the robot left as specified by the
 * number of degrees. It rotates the wheels as per the power specified.
 */
void turnLeftDeg(int degrees, int power){

  // reset encoder counts
  masterCount = 0;
  slaveCount = 0;

  // determine the tick goal
  int tickGoal = turnTickGoalCalculator(degrees);

  // Start the motors in a left point turn
  setMasterMotorSpeed(-1 * power);
  setSlaveMotorSpeed(power);

  // Since the wheels may go at slightly different speeds due to 
  // manufacturing tolerances, etc., we need to test both encoders 
  // and control both motors separately. This may result in one motor
  // going for longer than another but it will ultimately result 
  // in a much more accurate turn.
  while(slaveCount < tickGoal || masterCount > -1 * tickGoal){
    
    if(slaveCount > tickGoal){
      setSlaveMotorSpeed(0);
    }

    if(masterCount < -1 * tickGoal){
      setMasterMotorSpeed(0);
    }
  }

  // make sure both motors stop after the loop
  setSlaveMotorSpeed(0);
  setMasterMotorSpeed(0);
}

/**
 * This fuctions rotates the robot right as specified by the
 * number of degrees. It rotates the wheels as per the power specified.
 */
void turnRightDeg(int degrees, int power){

  // reset encoder counts
  masterCount = 0;
  slaveCount = 0;

  // determine the tick goal
  int tickGoal = turnTickGoalCalculator(degrees);

  // Start the motors in a right point turn
  setMasterMotorSpeed(power);
  setSlaveMotorSpeed(-1 * power);

  // Since the wheels may go at slightly different speeds due to 
  // manufacturing tolerances, etc., we need to test both encoders 
  // and control both motors separately. This may result in one motor
  // going for longer than another but it will ultimately result 
  // in a much more accurate turn.
  while(masterCount < tickGoal || slaveCount > -1 * tickGoal){
    
    if(masterCount > tickGoal){
      setMasterMotorSpeed(0);
    }

    if(slaveCount < -1 * tickGoal){
      setSlaveMotorSpeed(0);
    }
  }

  // make sure both motors stop after the loop
  setSlaveMotorSpeed(0);
  setMasterMotorSpeed(0);
}

/**
 * This function moves the robot X milimetres in a straightline.
 * Where X is the parameter, mmToTravel. masterPower is the
 * starting power for the left motor. The slavePower (right)
 * is given an initial power relative to this. This difference may
 * need to be tweaked.
 * 
 * For example, if mmToTravel = 100, and masterPower = 30
 * We would travel 100 mm (10 cm) with the master motor
 * set to a power of 30.
 */
void driveStraightDistance(int mmToTravel, int masterPower){
  
  int tickGoal = straightTickGoalCalculator(mmToTravel);

  // DEBUG TICKGOAL
  Serial.print("The mmToTravel is: ");
  Serial.print(mmToTravel);
  Serial.print(", The tick goal is: ");
  Serial.println(tickGoal);

  // This will count up the total encoder ticks despite the 
  // fact that the encoders are constantly reset.
  int totalTicks = 0;

  // Initialise slavePower as masterPower - 5 so we don't get 
  // huge error for the first few iterations. The -5 value is 
  // based off a rough guess of how much the motors are different,
  // which prevents the robot from veering off course at the start 
  // of the function.
  int slavePower = masterPower; 

  volatile int localErrorVar = 0;

  masterCount = 0;
  slaveCount = 0;
  
  while(absolute(totalTicks) < tickGoal){

    // Set the 2 motors to the masterPower and slavePower
    setSlaveMotorSpeed(masterPower);
    setMasterMotorSpeed(slavePower);

    // We calculate the error between the wheel.
    // A negative error means the slave wheel should slow down.
    // A positive error means the slave wheel should speed up.
    localErrorVar = masterCount - slaveCount;

    // DEBUG LOCAL ERROR VAR
    Serial.print("Error: ");
    Serial.print(localErrorVar);

    // Dividing by kp means that the error is scaled accordingly 
    // so that the motor value does not change too much or too 
    // little. You should 'tune' kp to get the best value. 
    slavePower = slavePower + localErrorVar / kp;

    // DEBUG SLAVE POWER
    Serial.print(", Slave power: ");
    Serial.println(slavePower);

    totalTicks = totalTicks + masterCount;

    // DEBUG TOTALTICKS
    Serial.print("The totalTicks so far: ");
    Serial.println(totalTicks);

    // Reset the encoders every loop so we have a fresh value 
    // to use to calculate the error.
    masterCount = 0;
    slaveCount = 0;

    // Makes the loop repeat ten times a second. If it repeats 
    // too much we lose accuracy due to the fact that we don't have
    // access to floating point math, however if it repeats to 
    // little the proportional algorithm will not be as effective.
    // Keep in mind that if this value is changed, kp must change accordingly.
    delay(200);
  }

  // Stop the wheels moving once we have reached the destination
  setSlaveMotorSpeed(0);
  setMasterMotorSpeed(0);
}


// ----------------------------------------------------------------
// ----------------------- Main Function --------------------------
// ----------------------------------------------------------------

void loop(){
  driveStraightDistance(500, 100);
  delay(1000);
  driveStraightDistance(500, -100);
  delay(1000);
}



// ----------------------------------------------------------------
// ------------------------- Protocol -----------------------------
// ----------------------------------------------------------------


/**
 * - Request Packet contains type, and 2 parameters for method
 * - type1, distance to travel in mm, power of master to go forward
 * - type2, degrees to rotate left, power to turn left
 * - type3, degrees to rotate right, power to turn right
 * - type4, distance to travel in mm, power of master to go backward
 * - type5, 0, 0, tells robot to stop
 * - Example: "one, 100, 30"
 * 
 * - Response packet contains type, success, error msg
 * - Example: "one, FAIL, unknown"
 */

void decodeProtocol(char packet[]){
  // need to define length

char test[] ="one, 100000, 30";
char * command;
command = strtok (test,",");
}


 


