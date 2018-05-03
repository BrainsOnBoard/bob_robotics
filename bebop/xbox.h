#pragma once

#include <cstdint>

#include <iostream>
#include <thread>

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>

namespace Xbox {
using namespace std;

enum Button
{
    ButtonA = 0,
    ButtonB = 1,
    ButtonX = 2,
    ButtonY = 3,
    ButtonLB = 4,
    ButtonRB = 5,
    ButtonBack = 6,
    ButtonStart = 7,
    ButtonXbox = 8,
    ButtonLStick = 9,
    ButtonRStick = 10,
    ButtonLeft = 11,
    ButtonRight = 12,
    ButtonUp = 13,
    ButtonDown = 14
};

enum Axis
{
    AxisLStickH = 0,
    AxisLStickV = 1,
    AxisRStickH = 3,
    AxisRStickV = 4,
    AxisLTrigger = 2,
    AxisRTrigger = 5,
    AxisDpadH = 6,
    AxisDpadV = 7
};

using xthreadcb = void (*)(js_event *, void *);

class Controller
{
public:
    ~Controller()
    {
        close();
    }

    bool open()
    {
        fd = ::open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
        return fd >= 0;
    }

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

    void start_thread(xthreadcb callback, void *data)
    {
        if (!rthread) {
            js = new js_event;
            rthread = new thread(run_thread, this, callback, data);
        }
    }

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

        js.type &= ~JS_EVENT_INIT;

        if (js.type == JS_EVENT_AXIS &&
            (js.number <= AxisLStickV || js.number == AxisRStickH ||
             js.number == AxisRStickV) &&
            abs(js.value) < deadzone) {
            js.value = 0;
        }

        return true;
    }

    static const char *get_button_name(__u8 number)
    {
        switch (number) {
        case ButtonA:
            return "A";
        case ButtonB:
            return "B";
        case ButtonX:
            return "X";
        case ButtonY:
            return "Y";
        case ButtonLB:
            return "LB";
        case ButtonRB:
            return "RB";
        case ButtonBack:
            return "BACK";
        case ButtonStart:
            return "START";
        case ButtonXbox:
            return "XBOX";
        case ButtonLStick:
            return "LSTICK";
        case ButtonRStick:
            return "RSTICK";
        case ButtonLeft:
            return "LEFT";
        case ButtonRight:
            return "RIGHT";
        case ButtonUp:
            return "UP";
        case ButtonDown:
            return "DOWN";
        }
        return "(unknown)";
    }

    static const char *get_axis_name(__u8 number)
    {
        switch (number) {
        case AxisLStickH:
            return "LSTICKH";
        case AxisLStickV:
            return "LSTICKV";
        case AxisRStickH:
            return "RSTICKH";
        case AxisRStickV:
            return "RSTICKV";
        case AxisLTrigger:
            return "LTRIGGER";
        case AxisRTrigger:
            return "RTRIGGER";
        case AxisDpadH:
            return "DPADH";
        case AxisDpadV:
            return "DPADV";
        }
        return "(unknown)";
    }

private:
    int fd = 0;
    thread *rthread = nullptr;
    bool closing = false;
    js_event *js = nullptr;
    static const __s16 deadzone = 10000;
    static const long sleepmillis = 25;

    static void run_thread(Controller *c, xthreadcb callback, void *data)
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
