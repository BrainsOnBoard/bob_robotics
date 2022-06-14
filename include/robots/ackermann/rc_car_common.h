#pragma once

namespace BoBRobotics {
namespace Robots {
namespace Ackermann {
namespace RCCar {

#define RCCAR_SLAVE_ADDRESS 0x29

enum class State : uint8_t
{
    RemoteControl,
    I2CControl
};

enum class Command : uint8_t
{
    SetStateRemoteControl,
    SetStateI2CControl,
    ReadRemoteControl,
    Drive
};

struct __attribute__((packed)) Movement
{
    int8_t speed, turn;
};

struct __attribute__((packed)) Message
{
    Command command;
    Movement move;
};
} // RCCar
} // Ackermann
} // Robots
} // BoBRobotics
