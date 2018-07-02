/* 
This is a simple sketch based on the Adafruit DC motor test that controls the 
2Dbee's four motors. The motor direction is set using the bits in the first byte
transmitted, then the remaining 4 bytes set the individual motor speeds.
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Assign motors to ports
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *rearRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rearLeftMotor = AFMS.getMotor(4);

Servo pan;
Servo tilt;

void setup() {
  //init serial communication at 9600 bits per sec
  Serial.begin(9600); 

  // setup i2c with slave address
  Wire.begin(SLAVE_ADDRESS);

  // if Master requests data -> call requestEvent() 
  Wire.onRequest(requestEvent);

  // if Master sends data -> call receiveEvent()
  Wire.onReceive(receiveEvent);

  pan.attach(9);
  //tilt.attach(10);

  // centre
  pan.write(90);

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  rearRightMotor->setSpeed(0);
  rearRightMotor->run(FORWARD);
  rearLeftMotor->setSpeed(0);
  rearLeftMotor->run(FORWARD);
  frontRightMotor->setSpeed(0);
  frontRightMotor->run(FORWARD);
  frontLeftMotor->setSpeed(0);
  frontLeftMotor->run(FORWARD);
  
  // turn on motor
  rearRightMotor->run(RELEASE);
  rearLeftMotor->run(RELEASE);
  frontRightMotor->run(RELEASE);
  frontLeftMotor->run(RELEASE);
}

void requestEvent() 
{
	// do nothing for now...
}

void receiveEvent(int numBytes) 
{

  // check incoming bytes from master
  uint8_t read_array[4]; // should be 4 values for the 4 motors
  int i = 0;
  while(Wire.available()) {
    read_array[i] = Wire.read();
    i++;
  }

  // If attached motor input has changed
  if(rightAttached && read_array[1] != movement[1]) {
    movement[1] = read_array[1];
    updateMotors = true;
  }
  
  if (m1 > 127) {
    frontRightMotor->run(FORWARD);
	frontRightMotor->setSpeed((read_array[0]-127)*2);  
  } else {
    frontRightMotor->run(BACKWARD);
	frontRightMotor->setSpeed(255-(read_array[0]*2));  
  }
  if (m2 > 127) {
    frontLeftMotor->run(FORWARD);
	frontLeftMotor->setSpeed((read_array[1]-127)*2);  
  } else {
    frontLeftMotor->run(BACKWARD);
	frontLeftMotor->setSpeed(255-(read_array[1]*2));  
  }
  if (m3 > 127) {
    rearRightMotor->run(BACKWARD);
	rearRightMotor->setSpeed((read_array[2]-127)*2);  
  } else {
    rearRightMotor->run(FORWARD);
	rearRightMotor->setSpeed(255-(read_array[2]*2));  
  } 
  if (m4 > 127) {
    rearLeftMotor->run(BACKWARD);
	rearLeftMotor->setSpeed((read_array[3]-127)*2);  
  } else {
    rearLeftMotor->run(FORWARD);
	rearLeftMotor->setSpeed(255-(read_array[3]*2));  
  }
  
}


void loop() {
	// NOT USED	
}
