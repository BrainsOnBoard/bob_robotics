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
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test! #2");

  //pan.attach(9);
  //tilt.attach(10);

  // centre
  //pan.write(90);

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // turn on motor
  rearRightMotor->run(RELEASE);
  rearLeftMotor->run(RELEASE);
  frontRightMotor->run(RELEASE);
  frontLeftMotor->run(RELEASE);


  rearRightMotor->setSpeed(0);
  rearRightMotor->run(FORWARD);
  rearLeftMotor->setSpeed(0);
  rearLeftMotor->run(FORWARD);
  frontRightMotor->setSpeed(0);
  frontRightMotor->run(FORWARD);
  frontLeftMotor->setSpeed(0);
  frontLeftMotor->run(FORWARD);

  Serial.println("Setup complete");

}

uint8_t j = 0;
int input;
byte motors = 0;
byte m1mask = 1;
byte m2mask = 2;
byte m3mask = 4;
byte m4mask = 8;

int has_serial = 1;
uint8_t in_data[8];

int cnt = 0;

void serialEvent(){

  //Serial.println("Serial in!");

  // buffer for data

  /*for (int k = 0; k < 8; ++k) {
    in_data[k] = 0;
  }*/

  int i = 0;

  uint8_t input;

  if (Serial.available() > 8) {
    input = Serial.read();
    while (input != 255) {
      in_data[i] = input;
      ++i;
      input = Serial.read();
    }

    if (i > 7) {

    byte m1 = in_data[0];
    byte m2 = in_data[1];
    byte m3 = in_data[2];
    byte m4 = in_data[3];
    if (m1 > 0) {
      frontRightMotor->run(FORWARD);
    } else {
      frontRightMotor->run(BACKWARD);
    }
    j=in_data[4];
    frontRightMotor->setSpeed(j);
    if (m2 > 0) {
      frontLeftMotor->run(FORWARD);
    } else {
      frontLeftMotor->run(BACKWARD);
    }
    j=in_data[5];
    frontLeftMotor->setSpeed(j);
    if (m3 > 0) {
      rearRightMotor->run(BACKWARD);
    } else {
      rearRightMotor->run(FORWARD);
    }
    j=in_data[6];
    rearRightMotor->setSpeed(j);
    if (m4 > 0) {
      rearLeftMotor->run(BACKWARD);
    } else {
      rearLeftMotor->run(FORWARD);
    }
    j=in_data[7];
    rearLeftMotor->setSpeed(j);
    //input=Serial.read();
    //pan.write(input);
    }
    has_serial = 1;
   }

}

void loop() {

  if (has_serial == 0) {
    cnt++;
    if (cnt > 10) {
      rearRightMotor->setSpeed(0);
      rearRightMotor->run(FORWARD);
      rearLeftMotor->setSpeed(0);
      rearLeftMotor->run(FORWARD);
      frontRightMotor->setSpeed(0);
      frontRightMotor->run(FORWARD);
      frontLeftMotor->setSpeed(0);
      frontLeftMotor->run(FORWARD);

      rearRightMotor->run(RELEASE);
      rearLeftMotor->run(RELEASE);
      frontRightMotor->run(RELEASE);
      frontLeftMotor->run(RELEASE);
    }
  } else {
    cnt = 0;
  }
  has_serial = 0;

  //Serial.println("loop...");
  delay(3);

}
