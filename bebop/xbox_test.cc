#include "xbox.h"

using namespace std;
using namespace Xbox;

void
handle_button(js_event &js)
{
    const char *name = Controller::get_button_name(js.number);
    if (js.value) {
        cout << "Button pushed: " << name << " (" << (int) js.number << ")"
             << endl;
    } else {
        cout << "Button released: " << name << " (" << (int) js.number << ")"
             << endl;
    }
}

void
handle_axis(js_event &js)
{
    const char *name = Controller::get_axis_name(js.number);
    cout << "Axis " << name << " (" << (int) js.number << "): " << js.value
         << endl;
}

void
callback(js_event *js, void *)
{
    if (!js) {
        cerr << "Error reading from joystick" << endl;
        exit(1);
    }

    switch (js->type) {
    case JS_EVENT_BUTTON:
        handle_button(*js);
        break;
    case JS_EVENT_AXIS:
        handle_axis(*js);
    }
}

int
main()
{
    cout << "Xbox controller test program" << endl;
    cout << "Press return to quit" << endl << endl;

    Controller cont;
    if (!cont.open()) {
        cerr << "Error: Could not open joystick" << endl;
        return 1;
    }

    cout << "Opened joystick" << endl;
    cont.start_thread(callback, nullptr);

    cin.ignore();

    cont.close();
    cout << "Controller closed" << endl;

    return 0;
}
