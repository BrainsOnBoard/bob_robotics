#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace Joystick {
/*
 * Controller axes. Note that the triggers are axes as is the dpad. The dpad is
 * slightly strange in that it's also treated as buttons (i.e. pressing up gives
 * both a button event and an axis event).
 */
enum Axis
{
    LeftStickHorizontal = 0,
    LeftStickVertical = 1,
    RightStickHorizontal = 3,
    RightStickVertical = 4,
    LeftTrigger = 2,
    RightTrigger = 5,
    DpadHorizontal = 6,
    DpadVertical = 7
};

struct Event
{
    unsigned int number;
    int16_t value;
    bool isAxis;
    bool isInitial;
};

// For callbacks when a controller event occurs (button is pressed, axis moves)
using Callback = void (*)(Event *js, void *userData);

class JoystickBase
{
private:
    Event m_JsEvent;

protected:
    bool m_Closing = false;

public:
    virtual bool open() = 0;
    virtual bool read(Event &js) = 0;

    virtual ~JoystickBase()
    {
        close();
    }

    bool read()
    {
        return read(m_JsEvent);
    }

    /*
     * Get the name of the button corresponding to number.
     */
    std::string getButtonName(unsigned int number)
    {
        // these values should be defined before this header is included
        switch (number) {
        case A:
            return "A";
        case B:
            return "B";
        case X:
            return "X";
        case Y:
            return "Y";
        case LB:
            return "LB";
        case RB:
            return "RB";
        case Back:
            return "BACK";
        case Start:
            return "START";
        case LeftStickButton:
            return "LSTICK";
        case RightStickButton:
            return "RSTICK";
        case Left:
            return "LEFT";
        case Right:
            return "RIGHT";
        case Up:
            return "UP";
        case Down:
            return "DOWN";
        }
        return "(unknown)";
    }

    /*
     * Get the name of the axis corresponding to number.
     */
    std::string getAxisName(unsigned int number)
    {
        switch (number) {
        case LeftStickHorizontal:
            return "LSTICKH";
        case LeftStickVertical:
            return "LSTICKV";
        case RightStickHorizontal:
            return "RSTICKH";
        case RightStickVertical:
            return "RSTICKV";
        case LeftTrigger:
            return "LTRIGGER";
        case RightTrigger:
            return "RTRIGGER";
        case DpadHorizontal:
            return "DPADH";
        case DpadVertical:
            return "DPADV";
        }
        return "(unknown)";
    }

    /*
     * Start the read thread in the background. Call callback when an event
     * occurs.
     */
    void startThread(Callback callback, void *data)
    {
        if (!m_Thread) {
            m_Thread = std::unique_ptr<std::thread>(
                    new std::thread(runThread, this, callback, data));
        }
    }

    virtual void close()
    {
        if (m_Closing) {
            return;
        }
        m_Closing = true;

        if (m_Thread) {
            m_Thread->join();
        }
    }

private:
    std::unique_ptr<std::thread> m_Thread;

    /*
     * This function is invoked by the read thread. It repeatedly polls the
     * controller, calling the callback function as appropriate. If an error
     * occurs, the callback is called with a nullptr in place of the js_event
     * struct.
     */
    static void runThread(JoystickBase *c, Callback callback, void *userData)
    {
        while (c->read()) {
            callback(&c->m_JsEvent, userData);
        }
        if (!c->m_Closing) {
            callback(nullptr, userData);
        }
    }
};
}
