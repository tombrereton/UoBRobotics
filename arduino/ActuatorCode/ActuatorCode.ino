  /*
   * actuator firmware 
   * 
   */
   
  #include <Servo.h>
  
  const int servoGrip = 8;    // location of first servo
  const int servoArm = 10;    // location of second servo 
  
  Servo grip;     // create servo object to control the grip servo
  Servo arm;      // create servo object to control the arm of the servo that will move the arm up and down 
  
  int potpin = 0;    // analog pin used to connect the potentiometer
  int servoVal;      // variable to read the value from the analog pin
  int x;
  /*
   * setup method 
   */
  void setup() {
    
    grip.attach(servoGrip);  // attaches the servo on pin 9 to the servo object
    arm.attach(servoArm);  // attaches the up down servo to pin 10
  
    // example codes online used serial but not 100% sure why 
    Serial.begin(9600);
    
  }
  
  void loop() {
  //  servoVal = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  //  servoVal = map(servoVal, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    //arm.write(servoVal);                  // sets the servo position according to the scaled value
       
    // if serial is available run , check if incomoing bite is a c  if it is c shut or close if o go back 10 
    if(Serial.available())
    {
      int incomingByte = Serial.read();
  
      if(incomingByte == 'c'){
        delay(300);
        gripHard();
      }
  
      if(incomingByte == 'o'){
      delay(300);                
      gripRelease();
      }

      if(incomingByte == 'u'){
      delay(300);                
      armUp();
      }

      if(incomingByte == 'd'){
      delay(300);                
      armDown();
      }
    }

    delay(500);
    armDown();
    delay(500);
    armUp();
    delay(500);
  }
  
  /*
   * method to grip the arm - uses the grip servo
   */
  void gripHard() {
  
    servoVal = analogRead(potpin);
    grip.write(100);
    
  }
  
  /*
   * method to relase the grip - uses servo grip
   * do not set to 0 oe 10 
   */
  void gripRelease(){
  
    grip.write(10); 
   
  }
  
  /*
   * method to move the arm up - uses servo arm
   */
  void armUp(){
  
    arm.write(150);
  }
  
  /*
   * method to move the arm down - uses servo arm 
   */
  void armDown(){
  
    arm.write(20);
  }
  
  
  

