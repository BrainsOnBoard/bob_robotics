#include "xbox.h"

namespace Parrot {
BebopXbox::BebopXbox(Bebop *bebop)
{
    if (!xbox.open()) {
        std::cerr << "Error: Could not find Xbox controller" << endl;
        return;
    }

    this->bebop = bebop;

    xbox.startThread(jsEventCallback, this);
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
    case Xbox::A:
        bebop->takeOff();
        break;
    case Xbox::B:
        bebop->land();
        break;
    }
}

void
BebopXbox::jsEventAxis(js_event *js)
{
    float f;

    switch (js->number) {
    case Xbox::RightStickHorizontal:
        f = maxbank * (float) (js->value) /
            (float) numeric_limits<__s16>::max();
        bebop->setRoll((i8) f);
        break;
    case Xbox::RightStickVertical:
        f = maxbank * (float) (-js->value) /
            (float) numeric_limits<__s16>::max();
        bebop->setPitch((i8) f);
        break;
    case Xbox::LeftStickHorizontal:
        f = maxyaw * (float) (js->value) / (float) numeric_limits<__s16>::max();
        bebop->setYaw((i8) f);
        break;
    case Xbox::LeftStickVertical:
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
