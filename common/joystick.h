#pragma once

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

// Standard C includes
#include <cstdint>
#include <cstring>
#include <cmath>

// POSIX includes
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>

// GeNN robotics includes
#include "../robots/motor.h"

using namespace GeNNRobotics::Robots;

namespace GeNNRobotics {
//----------------------------------------------------------------------------
// Joystick
//----------------------------------------------------------------------------
// Simple wrapper around Linux joystick to allow simple control of e.g. robots
class Joystick
{
public:
    Joystick(const char *device = "/dev/input/js0")
    {
        std::fill(std::begin(m_AxisState), std::end(m_AxisState), 0);
        if(!open(device)) {
            throw std::runtime_error("Cannot open joystick");
        }
    }

    ~Joystick()
    {
       if(m_Joystick >= 0) {
           close(m_Joystick);
       }
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool open(const char *device)
    {
        // Open joystick for non-blocking IO
        m_Joystick = ::open(device, O_RDONLY | O_NONBLOCK);
        if (m_Joystick < 0) {
            std::cerr << "Could not open joystick device '" << device << "' (" << strerror(errno) << ")" << std::endl;
            return false;
        }
        else {
            // Read number of buttons and axes
            char numButtons;
            char numAxes;
            if(ioctl(m_Joystick, JSIOCGBUTTONS, &numButtons) < 0 || ioctl(m_Joystick, JSIOCGAXES, &numAxes) < 0) {
                std::cerr << "Couldn't query joystick axes/buttons (" << strerror(errno) << ")" << std::endl;
                return false;
            }
            else {
                std::cout << "Opened " << (int)numAxes << " axis, " << (int)numButtons << " button joystick" << std::endl;
                m_AxisState.resize(numAxes, 0);
                m_ButtonState.resize(numButtons, 0);
                return true;
            }
        }
    }

    bool read()
    {
        // Clear all press and release bits
        for(uint8_t &s : m_ButtonState) {
            s &= ~(StatePress | StateRelease);
        }

        // **TODO** clear all press and release bits
        while(true) {
            // Attempt to read event from joystick device (non-blocking)
            js_event event;
            const ssize_t bytesRead = ::read(m_Joystick, &event, sizeof(js_event));

            // If there was an error
            if(bytesRead == -1) {
                // If there are no more events, return true
                if(errno == EAGAIN) {
                    return true;
                }
                // Otherwise return false
                else {
                    std::cerr << "Error: Could not read from joystick (" << strerror(errno) << ")" << std::endl;
                    return false;
                }
            }
            // Otherwise, if an event was read
            else if(bytesRead == sizeof(js_event)){
                // If event is axis, copy value into axis
                // **NOTE** initial state is specified by ORing these
                // types with JS_EVENT_INIT so this test handles initial state too
                if((event.type & JS_EVENT_AXIS) != 0) {
                    m_AxisState[event.number] = event.value;
                }
                // Otherwise, if a button state has changed
                else if((event.type & JS_EVENT_BUTTON) != 0) {
                    // Release
                    uint8_t &s = m_ButtonState[event.number];
                    if(event.value == 0) {
                        // Clear down
                        s &= ~StateDown;

                        // Set release
                        s |= StateRelease;
                    }
                    // Press
                    else {
                        // Set down and press
                        s |= (StateDown | StatePress);
                    }
                }
                else {
                    std::cerr << "Unknown event type " << (unsigned int)event.type << std::endl;
                    continue;
                }
            }
            else {
                std::cerr << "Unknown error" << std::endl;
                return false;
            }
        }

    }

    bool isButtonDown(uint8_t button)
    {
        return ((m_ButtonState[button] & StateDown) != 0);
    }

    bool isButtonPressed(uint8_t button)
    {
        return ((m_ButtonState[button] & StatePress) != 0);
    }

    bool isButtonReleased(uint8_t button)
    {
        return ((m_ButtonState[button] & StateRelease) != 0);
    }

    float getAxisState(uint8_t axis)
    {
        return (float)m_AxisState[axis] / (float)std::numeric_limits<int16_t>::max();
    }

    void drive(Motor &motor, float deadzone)
    {
        constexpr float pi = 3.141592653589793238462643383279502884f;
        constexpr float halfPi = pi / 2.0f;

        // Read joystick axis state and drive robot manually
        const float joystickX = getAxisState(0);
        const float joystickY = getAxisState(1);
        const bool deadX = (fabs(joystickX) < deadzone);
        const bool deadY = (fabs(joystickY) < deadzone);

        if(deadX && deadY) {
            motor.tank(0.0f, 0.0f);
        }
        else if(deadX) {
            motor.tank(-joystickY, -joystickY);
        }
        else if(deadY) {
            motor.tank(joystickX, -joystickX);
        }
        else {
            // If length of joystick vector places it in deadzone, stop motors
            const float r = sqrt((joystickX * joystickX) + (joystickY * joystickY));
            const float theta = atan2(joystickX, -joystickY);
            const float twoTheta = 2.0f * theta;

            // Drive motor
            if(theta >= 0.0f && theta < halfPi) {
                motor.tank(r, r * cos(twoTheta));
            }
            else if(theta >= halfPi && theta < pi) {
                motor.tank(-r * cos(twoTheta), -r);
            }
            else if(theta < 0.0f && theta >= -halfPi) {
                motor.tank(r * cos(twoTheta), r);
            }
            else if(theta < -halfPi && theta >= -pi) {
                motor.tank(-r, -r * cos(twoTheta));

            }

        }
    }

private:
    //----------------------------------------------------------------------------
    // Enumerations
    //----------------------------------------------------------------------------
    enum State
    {
        StateDown       = (1 << 0),
        StatePress      = (1 << 1),
        StateRelease    = (1 << 2)
    };

    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    int m_Joystick;

    std::vector<int16_t> m_AxisState;
    std::vector<uint8_t> m_ButtonState;
};
} // GeNNRobotics
