#pragma once

#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

namespace Xbox {

/*
 * Controller buttons. The left stick and right stick are also buttons (you can
 * click them.)
 */
enum Button
{
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LB = 4,
    RB = 5,
    Back = 6,
    Start = 7,
    XboxButton = 8,
    LeftStickButton = 9,
    RightStickButton = 10,
    Left = 11,
    Right = 12,
    Up = 13,
    Down = 14
};

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

// For callbacks when a controller event occurs (button is pressed, axis moves)
using ControllerCallback = void (*)(js_event *, void *);

class Controller
{
public:
    ~Controller()
    {
        close();
    }

    /*
     * Open connection to controller. Return true if connected successfully,
     * false otherwise.
     */
    bool open()
    {
        fd = ::open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
        return fd >= 0;
    }

    /*
     * Close connection to controller.
     */
    void close()
    {
        if (closing) {
            return;
        }

        closing = true;

        if (rthread) {
            rthread->join();
            delete rthread;
            delete js;
        }

        ::close(fd);
    }

    /*
     * Start the read thread in the background. Call callback when an event
     * occurs.
     */
    void startThread(ControllerCallback callback, void *data)
    {
        if (!rthread) {
            js = new js_event;
            rthread = new std::thread(runThread, this, callback, data);
        }
    }

    /*
     * Read controller event into js struct. Returns true if read successfully,
     * false if an error occurs.
     */
    bool read(js_event &js)
    {
        while (!closing) {
            const ssize_t bytes = ::read(fd, &js, sizeof(js));
            if (bytes > 0) {
                break;
            }
            if (errno != EAGAIN) {
                return false;
            }

            usleep(sleepmillis * 1000);
        }
        if (closing) {
            return false;
        }

        // we treat initial events (indicating initial state of controller) like
        // any other
        js.type &= ~JS_EVENT_INIT;

        // if it's an axis event for the left or right stick, account for
        // deadzone
        if (js.type == JS_EVENT_AXIS && js.number >= LeftStickHorizontal &&
            js.number <= RightStickVertical && abs(js.value) < deadzone) {
            js.value = 0;
        }

        return true;
    }

    /*
     * Get the name of the button corresponding to number.
     */
    static const char *getButtonName(__u8 number)
    {
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
        case XboxButton:
            return "XBOX";
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
    static const char *getAxisName(__u8 number)
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

private:
    int fd = 0;                          // file descriptor for joystick device
    std::thread *rthread = nullptr;           // read thread object
    bool closing = false;                // is controller closing?
    js_event *js = nullptr;              // struct to contain joystick event
    static const __s16 deadzone = 10000; // size of deadzone for axes (i.e.
                                         // region within which not activated)
    static const long sleepmillis = 25;  // number of milliseconds between polls

    /*
     * This function is invoked by the read thread. It repeatedly polls the
     * controller, calling the callback function as appropriate. If an error
     * occurs, the callback is called with a nullptr in place of the js_event
     * struct.
     */
    static void runThread(Controller *c,
                          ControllerCallback callback,
                          void *data)
    {
        while (c->read(*c->js)) {
            callback(c->js, data);
        }
        if (!c->closing) {
            callback(nullptr, data);
        }
    }
};
}
