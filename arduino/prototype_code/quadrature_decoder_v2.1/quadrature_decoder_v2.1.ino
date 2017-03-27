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


// The simplest approach to estimate velocity = Δpos/Δt: 
// measure the change in position and divide by the change in time. 


// ----------------------------------------------------------------
// ----------------------- Libraries ------------------------------
// ----------------------------------------------------------------


// ----------------------------------------------------------------
// ----------------------- Global Variables -----------------------
// ----------------------------------------------------------------

// pulse counter
// The pulse counter increments by one for every 1/360 rotation (1 degree)
// (CONFIRM last statement)
volatile long masterCount = 0;
volatile long slaveCount = 0;

// Variables for calculating distance
const double PIE = 3.1415;
const double WHEEL_DIAMETER_CM = 9.5;
const double CIRCUMFERENCE_CM = PIE * WHEEL_DIAMETER_CM;
const int TICKS_PER_CM = 360 / CIRCUMFERENCE_CM;

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
// -------------------------- Functions ---------------------------
// ----------------------------------------------------------------

void setup() {
  // read in from pins 7 & 8 for master wheel
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);

  // read in from pins 5 & 6 for slave wheel
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);

  // init serial
  // We use 250000 for a high speed data transfer
//  Serial.begin(250000);
  Serial.begin(9600);

  // We attach the encoders for the master wheel (left)
  attachInterrupt(7, masterEncoderA, intMode);
  attachInterrupt(8, masterEncoderB, intMode);

  // We attach the encoders for the slave wheel (right)
  attachInterrupt(5, slaveEncoderA, intMode);
  attachInterrupt(6, slaveEncoderB, intMode);
}

// ----------------------------------------------------------------
// --------- Interrupt Service Routines Master Wheel --------------
// ----------------------------------------------------------------

/** 
 *  masterEncoderA for the master wheel.
 *  If we get a read from pin 7 or 8 but not both, we decrease the 
 *  masterCount, otherwise increase.
 */
void masterEncoderA () {
  // decode quadrature
  (digitalRead(7) ^ digitalRead(8)) ? masterCount-- : masterCount++;
  // debug to serial
//  Serial.println(masterCount);
}
  
/** 
 *  masterEncoderB for the master wheel.
 *  If we get a read from pin 7 or 8 but not both, we increase the 
 *  masterCount, otherwise decrease.
 */
void masterEncoderB () {
  // decode quadrature
  (digitalRead(7) ^ digitalRead(8)) ? masterCount++ : masterCount--;
  // debug to serial
//  Serial.println(masterCount);
}

// ----------------------------------------------------------------
// --------- Interrupt Service Routines Slave Wheel --------------
// ----------------------------------------------------------------

/** 
 *  slaveEncoderA for the slave wheel.
 *  If we get a read from pin 5 or 6 but not both, we decrease the 
 *  slaveCount, otherwise increase.
 */
void slaveEncoderA () {
  // decode quadrature
  (digitalRead(5) ^ digitalRead(6)) ? slaveCount-- : slaveCount++;
}
  
/** 
 *  slaveEncoderB for the slave wheel.
 *  If we get a read from pin 7 or 8 but not both, we increase the 
 *  slaveCount, otherwise decrease.
 */
void slaveEncoderB () {
  // decode quadrature
  (digitalRead(5) ^ digitalRead(6)) ? slaveCount++ : slaveCount--;
}

// ----------------------------------------------------------------
// -------------------------- Functions ---------------------------
// ----------------------------------------------------------------

/**
 * This function determines the total number of ticks needed
 * to travel the milimetres specified to travel. It uses the formula:
 * 
 * Encoder ticks = (360 / circumference) * Distance to travel
 * or
 * Encoder ticks = TICKS_PER_CM * Distance to travel
 */
int tickGoalCalculator(int mmToTravel){
  int tickGoal = (TICKS_PER_CM * mmToTravel) / 10;
  return tickGoal;
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
  
  int tickGoal = tickGoalCalculator(mmToTravel);

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
  int slavePower = masterPower - 5; 

  volatile int localErrorVar = 0;

  masterCount = 0;
  slaveCount = 0;
  
  while(absolute(totalTicks) < tickGoal){

    // Set the 2 motors to the masterPower and slavePower
    // TODO: ADD FUNCTION

    // We calculate the error between the wheel.
    // A negative error means the slave wheel should slow down.
    // A positive error means the slave wheel should speed up.
    localErrorVar = masterCount - slaveCount;

    // DEBUG ERROR
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
  masterPower = 0;
  slavePower = 0;
  // TODO: function to set motor power
}

void loop(){
  driveStraightDistance(500, 30);
  delay(500);
  driveStraightDistance(500, -30);
}





// ----------------------------------------------------------------
// ---------------------- New Program Plan ------------------------
// ----------------------------------------------------------------

/** 
 * [x] Function to return sensor value of left encoder
 * [x] Function to return sensor value of right encoder
 * [] Function to specify power to master motor (left)
 * [] Function to specify power to slave motor (right)
 * [] Manually tune 'constant of proportionality,' kp
 * [x] Function to drive a certain distance straight
 * [] Function to turn left 90 degrees
 * [] Function to turn right 90 degrees
 */







