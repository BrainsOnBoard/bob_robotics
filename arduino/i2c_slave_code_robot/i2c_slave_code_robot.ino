#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x29 // slave address (arduino)

Servo servoLeft;                                // left motor
Servo servoRight;                               // right motor

volatile int movement[2];                       // left and right wheel movement
static volatile int changed;                    // will signal if any motor value has changed by the master
static volatile int is_attached;

// sensor values
volatile int sensor2602;                        // Air contaminants
volatile int sensor2610;                        // Iso-butane
volatile int sensor2611;                        // Methane
volatile int sensor2620;                        // Ethanol

// setup sensors and i2c connection
void setup() 
{

  //init serial communication at 9600 bits per sec
  Serial.begin(9600); 

  // setup i2c with slave address
  Wire.begin(SLAVE_ADDRESS);

  // if Master requests data -> call requestEvent() 
  Wire.onRequest(requestEvent);

  // if Master sends data -> call receiveEvent()
  Wire.onReceive(receiveEvent);
  
  // Attach servos
  servoLeft.attach(12);
  servoRight.attach(13); 
}

void loop() 
{

  // sensor reading  
  sensor2602 = analogRead(A0);  // Air contaminants
  sensor2610 = analogRead(A1);  // Iso-butane
  sensor2611 = analogRead(A2);  // Methane
  sensor2620 = analogRead(A2);  // Ethanol

  
}

// if Jetson requests data, we use this function to send.
// Here we map sensor values from a range to 0-255. This is 
// to ensure that the value will fit in 1 byte. It needs to be
// mentioned that different values of resistors 
// will have different effects on the readings, so if a resistor is
// changed, the mapping should be changed as well
void requestEvent() 
{
  
  uint8_t sensor_array[4]; 
  // mapping the values between 0-255 so we can transfer it in 1 byte
  sensor_array[0] = map(sensor2602, 0,  255,0,255);
  sensor_array[1] = map(sensor2610, 0,  255,0,255);
  sensor_array[2] = map(sensor2611, 0, 600,0,255);
  sensor_array[3] = map(sensor2620, 0, 255,0,255); // 680 is the maxish, but we don't need that much

  // send readings (i2c) to Master
  int result = Wire.write(sensor_array,4);
}


// Arduino receives motor commands from the Jetson using this function.
// We expect 2 bytes from from the Master in an array. array[0] is the
// left wheel and array[1] is the right wheel. If any of the 2 values 
// are 0, we stop the motors. Othewise, we update our global value (movement[]) 
// where we store the motor commands to use process later in the loop.
void receiveEvent(int numByte) 
{
  // check incoming bytes from master
  int8_t read_array[2]; // should be 2 values for the 2 servos
  int i=0;
  while(Wire.available()) {
    read_array[i]= Wire.read();
    i++;
  }

  servoLeft.writeMicroseconds(map(read_array[0], -128, 127, 1700, 1300));
  servoRight.writeMicroseconds(map(read_array[1], -128, 127, 1300, 1700));
}
