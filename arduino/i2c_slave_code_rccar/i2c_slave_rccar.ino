#include "Servo.h"
#include "Wire.h"

#define SLAVE_ADDRESS 0x29

volatile int movement[2];   // holds data about speed and turning
volatile bool updateMotors; // ture if we got commands from i2c
int remoteControlSpeed;     // signal from remote control (speed)
int remoteControlSteering;  // signal from remote control (steering)

const int PWMCONNECTED = 965;  // the minimum value of PWM when connected
const int NEUTRALVALUE = 1500; // neutral PWM value of speed (at this PWM the car is neutral)
const int MINCONTROL = 1050;   // minimum value of speed control
const int MAXCONTROL = 1900;   // maximum value of speed control
const int SPEEDERROR = 20;     // error limit when checking if controller active
const int TURNINGERROR = 5;    // error limit when checking if remote controller active

// arduino I/O pins
const byte STEERING = 3;
const byte THROTTLE = 5;
const byte STEERING_CONTROL = 10;
const byte THROTTLE_CONTROL = 11;


Servo esc, steering;

void setup() {

    // data from remote controller to arduino
    pinMode(STEERING, INPUT);
    pinMode(THROTTLE, INPUT);

    // setup servos
    esc.attach(THROTTLE_CONTROL);      //attach esc to pin
    steering.attach(STEERING_CONTROL); //attach steering to pin

    // if Master sends data -> call receiveEvent()
    Wire.onReceive(receiveEvent);
}

void loop() {
    // if there is incoming command from the remote controller, we block other controls
    if (checkRemote()) {
        updateMotors = false;
    }

    if (updateMotors) {  // if we have incoming input from i2c (and no remote control command)
        int mappedESC = map(movement[0], 0, 255, MINCONTROL, MAXCONTROL);
        esc.writeMicroseconds(mappedESC);
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
    if (remoteControlSpeed < PWMCONNECTED) {
        return false;
    }

    // if value is not the neutral value (+- some error), return true
    if (remoteControlSpeed <= NEUTRALVALUE -SPEEDERROR || // allow some error
        remoteControlSpeed >= NEUTRALVALUE +SPEEDERROR|| // allow some error
        remoteControlSteering <= 90+TURNINGERROR ||
        remoteControlSteering >= 90-TURNINGERROR )
    {
        return true;
    }
    return false;

}

// called when data is sent through i2c to arduino
// to update the motor values
void receiveEvent() {
    // check incoming bytes from master
    uint8_t read_array[2];
    while(Wire.available()) {
        read_array[i] = Wire.read();
    }
    movement[0] = read_array[0]; // throttle value
    movement[1] = read_array[1]; // steering value
    updateMotors = true;
}

