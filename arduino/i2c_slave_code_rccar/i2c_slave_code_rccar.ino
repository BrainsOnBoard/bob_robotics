#include "rc_car_common.h"

#include "Servo.h"
#include "Wire.h"

using namespace BoBRobotics::Robots::RCCar;

volatile Movement i2cMove{}, remoteMove{}; // holds data about speed and turning
volatile bool sendRemote = false;
volatile State state = State::RemoteControl;

const int MINCONTROL = 900;  // minimum value of speed control
const int MAXCONTROL = 1990; // maximum value of speed control
const int MINSTEERING = 860;
const int MAXSTEERING = 1870;

// arduino I/O pins
const byte STEERING = 3;
const byte THROTTLE = 5;
const byte STEERING_CONTROL = 10;
const byte THROTTLE_CONTROL = 11;

Servo esc, steering;

void
setup()
{
  // data from remote controller to arduino
  pinMode(STEERING, INPUT);
  pinMode(THROTTLE, INPUT);

  // setup servos
  esc.attach(THROTTLE_CONTROL);      //attach esc to pin
  steering.attach(STEERING_CONTROL); //attach steering to pin

  // starts i2c connection
  Wire.begin(RCCAR_SLAVE_ADDRESS);

  // if Master sends data -> call receiveEvent()
  Wire.onReceive(receiveEvent);
  Wire.onRequest(sendEvent);

  Serial.begin(9600);
  Serial.println("Initialised");
}

void
loop()
{
  const long t0 = millis();
  readRemote();

  if (state == State::RemoteControl) {
    drive(remoteMove);
  } else {
    // these have been updated in readEvent()
    drive(i2cMove);
  }

  // the esc needs a pulse every 20 ms to work
  constexpr long senddelay = 60;
  const long elapsed = (long) millis() - t0;
  if (elapsed <= senddelay) {
    delay(senddelay - elapsed);
  }
}

void
drive(volatile Movement &m)
{
  m.speed = constrain(m.speed, (int8_t) -100, (int8_t) 100);
  m.turn = constrain(m.turn, (int8_t) -100, (int8_t) 100);

  esc.writeMicroseconds(map(m.speed, -100, 100, MINCONTROL, MAXCONTROL));
  steering.write(map(m.turn, -100, 100, MINSTEERING, MAXSTEERING));
}

void
readRemote()
{
  int curTurn = pulseIn(STEERING, HIGH, 0);
  int curSpeed = pulseIn(THROTTLE, HIGH, 0);

  // curTurn is zero if controller not connected
  if (curTurn == 0) {
    remoteMove.speed = 0;
    remoteMove.turn = 0;
    return;
  }

  remoteMove.turn = map(curTurn, MINSTEERING, MAXSTEERING, -100, 100);
  remoteMove.speed = map(curSpeed, MINCONTROL, MAXCONTROL, -100, 100);
}

// called when data is sent through i2c to arduino
// to update the motor values
void
receiveEvent(int bytes)
{
  if (bytes != sizeof(Message)) {
    Serial.println("Error: bad message size: " + String(bytes));
    return;
  }

  // check incoming bytes from master
  union
  {
    uint8_t read_array[sizeof(Message)];
    Message msg;
  };

  int numBytes = 0; // number of bytes read
  while (Wire.available() && numBytes < bytes) {
    read_array[numBytes++] = Wire.read();
  }

  if (numBytes != sizeof(Message)) { // check we read all the data
    Serial.println("Error: Only read " + String(numBytes) + " bytes");
    return;
  }

  switch (msg.command) {
    case Command::SetState:
      Serial.println("Set state to: " + String((int) msg.state));
      state = msg.state;
      break;
    case Command::ReadRemoteControl:
      sendRemote = true;
      break;
    case Command::Drive:
      i2cMove.speed = msg.move.speed;
      i2cMove.turn = msg.move.turn;
      break;
  }
}

void
sendEvent()
{
  if (sendRemote) {
    const Movement m{ constrain(remoteMove.speed, (int8_t) -100, (int8_t) 100),
                      constrain(remoteMove.turn, (int8_t) -100, (int8_t) 100) };
    Wire.write(reinterpret_cast<const uint8_t *>(&m), sizeof(m));
    sendRemote = false;
  }
}
