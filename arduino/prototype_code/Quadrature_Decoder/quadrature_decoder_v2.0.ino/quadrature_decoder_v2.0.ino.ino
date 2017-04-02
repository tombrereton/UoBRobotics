// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// %%%%%%%%%%%%% Quadrature Decoder for EMG30 Motors %%%%%%%%%%%%%%
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// © UBRobotics 2016
// Written by Ed Chamberlain
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
// ----------------------------------------------------------------
// ----------------------- About this Program ---------------------
// ----------------------------------------------------------------
// * Read in encoders from both wheels and keep robot straight 
//   via count difference == 0.
// * The Pi will send a packet via a protocol which tells the robot
//   to drive straight, 90 degree left, 90 degree right, backwards.
// * This program contains functions for straight, left, right,
//   backwards.
// * The packet sent from the Pi will call these functions.

// ----------------------------------------------------------------
// -------------------------- Libraries ---------------------------
// ----------------------------------------------------------------


// ----------------------------------------------------------------
// ----------------------- Global Variables -----------------------
// ----------------------------------------------------------------

// pulse counter
// The pulse counter increments by one for every 1/360 rotation (1 degree)
volatile  long count = 0;

// This tells the interrupt to run the method on any change
#define intMode CHANGE

//
int leftPin = 8;
int rightPin = 9;

// ----------------------------------------------------------------
// -------------------------- Functions ---------------------------
// ----------------------------------------------------------------

void setup() {
  // init pin modes, active pullups
  pinMode(leftPin, INPUT_PULLUP);
  pinMode(rightPin, INPUT_PULLUP);

  // init serial
  // We use 250000 for a high speed data transfer
//  Serial.begin(250000);
  Serial.begin(9600);


  // init interrupts.
  // We run ecnoderA whenever there is a change from reading in from pin 7
  attachInterrupt(leftPin, encoderA, intMode);
  attachInterrupt(rightPin, encoderB, intMode) ;
}

void loop() {
  // put your main code here, to run repeatedly:
}

// ----------------------------------------------------------------
// ----------------- Interrupt Service Routines -------------------
// ----------------------------------------------------------------

// We want to keep the difference in the count
// at 0 when moving in a straight line.


// EncoderA for wheel 1. If we get a read from pin 7 or 8 but not both, 
// we decrease the count, otherwise we increase the count
void encoderA () {
  // decode quadrature
  (digitalRead(leftPin) ^ digitalRead(rightPin)) ? count-- : count++;

  // debug to serial
  Serial.println(count);
}

// EncoderB for wheel 1. If we get a read from pin 7 or 8 but not both, 
// we increase the count, otherwise decrease.
void encoderB () {
  (digitalRead(leftPin) ^ digitalRead(rightPin)) ? count++ : count--;
  Serial.println(count);
}

// serial event will inturrupt whenever serial data is available

// i.e. if the pi gives an instruction to the robot (ie turn left) we 
// reset the counter on the arduino
void serialEvent() {
  if (Serial.read() == 'r') {
    // reset count when receiving 'r'
    count = 0;
    Serial.println(count);
  }
}



