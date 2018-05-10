#pragma once

#include "bebop.h"
#include "xbox_generic.h"

namespace Parrot {
using buttonEvent = bool (*)(js_event *js);

class BebopXbox
{
public:
    buttonEvent m_ButtonCallback = nullptr;
    BebopXbox(Bebop *bebop);

private:
    Parrot::Bebop *bebop;
    Xbox::Controller xbox;
    static constexpr float maxbank = 50;
    static constexpr float maxup = 50;
    static constexpr float maxyaw = 100;

    void jsEventButton(js_event *js);
    void jsEventAxis(js_event *js);

    static void jsEventCallback(js_event *js, void *data);
};
}
