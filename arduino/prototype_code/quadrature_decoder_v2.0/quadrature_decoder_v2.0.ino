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
volatile int error = 0;

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
// --------- Interrupt Service Routines Master Wheel --------------
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
 *  masterEncoderB for the master wheel.
 *  If we get a read from pin 7 or 8 but not both, we increase the 
 *  masterCount, otherwise decrease.
 */
void slaveEncoderB () {
  // decode quadrature
  (digitalRead(5) ^ digitalRead(6)) ? slaveCount++ : slaveCount--;
}

// ----------------------------------------------------------------
// -------------------------- Functions ---------------------------
// ----------------------------------------------------------------


// ----------------------------------------------------------------
// ---------------------- New Program Plan ------------------------
// ----------------------------------------------------------------

/** 
 * [] Function to return sensor value of left encoder
 * [] Function to return sensor value of right encoder
 * [] Function to specify power to master motor (left)
 * [] Function to specify power to slave motor (right)
 * [] Manually tune 'constant of proportionality,' kp
 * [] Function to drive a certain distance straight
 */

/** 
 * The power we give to both the motors. Where master is the left
 * and slave is the right. We keep the master at the same speed
 * and increase or decrease the slave speed as per the error
 * to keep the robot in a straight line.
 */
int masterPower = 30;
int slavePower = 30;


void loop() {
  
    // Set the 2 motors the initial power
    // TODO: ADD FUNCTION

    // We calculate the error between the wheel.
    // A negative error means the slave wheel should slow down.
    // A positive error means the slave wheel should speed up.
    error = masterCount - slaveCount;

    // DEBUG ERROR
    Serial.print("Error: ");
    Serial.print(error);

    // Dividing by kp means that the error is scaled accordingly 
    // so that the motor value does not change too much or too 
    // little. You should 'tune' kp to get the best value. 
    slavePower = slavePower + error / kp;

    // DEBUG SLAVE POWER
    Serial.print(", Slave power: ");
    Serial.println(slavePower);

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






