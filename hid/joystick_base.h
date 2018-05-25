#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace GeNNRobotics {
namespace HID {
/*
 * Controller axes. Note that the triggers are axes as is the dpad. The dpad is
 * slightly strange in that it's also treated as buttons (i.e. pressing up gives
 * both a button event and an axis event).
 */
enum class Axis
{
    LeftStickHorizontal = 0,
    LeftStickVertical = 1,
    RightStickHorizontal = 3,
    RightStickVertical = 4,
    LeftTrigger = 2,
    RightTrigger = 5,
    DpadHorizontal = 6,
    DpadVertical = 7,
    NOTAXIS
};

struct Event
{
    unsigned int number;
    int16_t value;
    bool isAxis;
    bool isInitial;

    Axis axis() const
    {
        return !isAxis ? Axis::NOTAXIS : static_cast<Axis>(number);
    }

    std::string axisName() const
    {
        switch (axis()) {
        case Axis::LeftStickHorizontal:
            return "LSTICKH";
        case Axis::LeftStickVertical:
            return "LSTICKV";
        case Axis::RightStickHorizontal:
            return "RSTICKH";
        case Axis::RightStickVertical:
            return "RSTICKV";
        case Axis::LeftTrigger:
            return "LTRIGGER";
        case Axis::RightTrigger:
            return "RTRIGGER";
        case Axis::DpadHorizontal:
            return "DPADH";
        case Axis::DpadVertical:
            return "DPADV";
        case Axis::NOTAXIS:
            return "(not axis)";
        default:
            return "(unknown)";
        }
    }

    Button button() const
    {
        return isAxis ? Button::NOTBUTTON : static_cast<Button>(number);
    }

    std::string buttonName() const
    {
        // these values should be defined before this header is included
        switch (button()) {
        case Button::A:
            return "A";
        case Button::B:
            return "B";
        case Button::X:
            return "X";
        case Button::Y:
            return "Y";
        case Button::LB:
            return "LB";
        case Button::RB:
            return "RB";
        case Button::Back:
            return "BACK";
        case Button::Start:
            return "START";
        case Button::LeftStickButton:
            return "LSTICK";
        case Button::RightStickButton:
            return "RSTICK";
        case Button::Left:
            return "LEFT";
        case Button::Right:
            return "RIGHT";
        case Button::Up:
            return "UP";
        case Button::Down:
            return "DOWN";
        case Button::NOTBUTTON:
            return "(not button)";
        default:
            return "(unknown)";        
        }
    }
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
}; // JoystickBase
} // HID
} // GeNNRobotics
