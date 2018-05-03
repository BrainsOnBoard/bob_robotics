#include "xbox.h"

namespace Parrot {
BebopXbox::BebopXbox(Bebop *bebop)
{
    if (!xbox.open()) {
        cerr << "Error: Could not find Xbox controller" << endl;
        return;
    }

    this->bebop = bebop;

    xbox.start_thread(jsEventCallback, this);
}

void
BebopXbox::jsEventButton(js_event *js)
{
    if (m_ButtonCallback && m_ButtonCallback(js)) {
        return;
    }
    if (!js->value) {
        return;
    }

    switch (js->number) {
    case Xbox::ButtonA:
        bebop->takeOff();
        break;
    case Xbox::ButtonB:
        bebop->land();
        break;
    }
}

void
BebopXbox::jsEventAxis(js_event *js)
{
    float f;

    switch (js->number) {
    case Xbox::AxisRStickH:
        f = maxbank * (float) (js->value) /
            (float) numeric_limits<__s16>::max();
        bebop->setRoll((i8) f);
        break;
    case Xbox::AxisRStickV:
        f = maxbank * (float) (-js->value) /
            (float) numeric_limits<__s16>::max();
        bebop->setPitch((i8) f);
        break;
    case Xbox::AxisLStickH:
        f = maxyaw * (float) (js->value) / (float) numeric_limits<__s16>::max();
        bebop->setYaw((i8) f);
        break;
    case Xbox::AxisLStickV:
        f = maxup * (float) (-js->value) / (float) numeric_limits<__s16>::max();
        bebop->setUpDown((i8) f);
        break;
    }
}

void
BebopXbox::jsEventCallback(js_event *js, void *data)
{
    Parrot::BebopXbox *cont = reinterpret_cast<Parrot::BebopXbox *>(data);

    switch (js->type) {
    case JS_EVENT_BUTTON:
        cont->jsEventButton(js);
        break;
    case JS_EVENT_AXIS:
        cont->jsEventAxis(js);
        break;
    }
}
}
