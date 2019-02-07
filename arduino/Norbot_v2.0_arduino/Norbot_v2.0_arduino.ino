
#include "Servo.h"
#include "Wire.h"

#define SLAVE_ADDRESS 0x29 

volatile int movement[2];   // holds data about speed and turning
volatile bool updateMotors; // ture if we got commands from i2c
int remoteControlSpeed;     // signal from remote control (speed)
int remoteControlSteering;  // signal from remote control (steering)

int neutralValue = 1500;
byte STEERING = 3;
byte THROTTLE = 5;

byte STEERING_CONTROL = 10;
byte THROTTLE_CONTROL = 11;


Servo esc, steering;
 
void setup() {

  pinMode(STEERING, INPUT);
  pinMode(THROTTLE, INPUT);
 
  esc.attach(THROTTLE_CONTROL);      //attach esc to pin 9
  steering.attach(STEERING_CONTROL); //attach steering to pin 6

  // if Master sends data -> call receiveEvent()
  Wire.onReceive(receiveEvent);

  // if Master requests data -> call requestEvent() 
  Wire.onRequest(requestEvent);
  
}
 
void loop() {


  // if there is incoming command from the remote controller, we block other controls
  if (checkRemote()) {
    updateMotors = false;
  }

  if (updateMotors) {  // if we have incoming input from i2c (and no remote control command)
    
    int mappedESC = map(movement[0], 0, 255, 1050, 1900); // 1050full backward, 1500 neutral, 1950 full forward
    esc.writeMicroseconds(mappedESC);

    // 55 left, 90 center, 125 right
    steering.write(movement[1]);

    updateMotors = false;

  // if remote control is active, we block commands from i2c and listen to the remote controller
  } else {
    esc.writeMicroseconds(remoteControlSpeed); 
    if (remoteControlSteering != 0) { 
      steering.write(remoteControlSteering);
    }
  }

  // the esc needs a pulse every 20 ms to work
  delay(20);
  
}

// checks if remote controller is in command
bool checkRemote() {

  // read PWM from receiver
  remoteControlSpeed = pulseIn(THROTTLE, HIGH);
  remoteControlSteering = pulseIn(STEERING, HIGH);

  // the receiver gives a PWM signal of around 950, when remote controller is not connected
  if (remoteControlSpeed < 965) {
    return false; 
  }
  
  // if value is not the neutral value (+- some error), return true
  if (remoteControlSpeed <= neutralValue -20 || // allow some error
      remoteControlSpeed >= neutralValue + 20|| // allow some error
      remoteControlSteering <= 90+5 || 
      remoteControlSteering >= 90-5 ) 
  {     
    return true;
  } 
  return false;
  
}


// called when data is sent through i2c to arduino
// to update the motor values
void receiveEvent(int numBytes) {
  // check incoming bytes from master
  uint8_t read_array[2];  
  int i=0;
  while(Wire.available()) {
    read_array[i] = Wire.read();
    i++;
  }

  // If motor input has changed
  if(read_array[0] != movement[0]) {
    movement[0] = read_array[0]; // throttle value
  }

  if(read_array[1] != movement[1]) {
    movement[1] = read_array[1]; // steering value
  }
  
  updateMotors = true;
}


void requestEvent() {
  // not used 
}
