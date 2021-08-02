#include "common.h"

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "hid/joystick.h"
#include "net/client.h"
#include "robots/tank/tank_netsink.h"
#include "vicon/udp.h"

int bobMain(int, char **)
{
    HID::Joystick joystick;

    // Connect to Vicon system
    Vicon::UDPClient<> vicon(51001);
    auto viconObject = vicon.getObjectReference(5s);

    // Make connection to robot on default port
    Net::Client client;

    // Transmit motor commands over network
    Robots::Tank::TankNetSink tank(client);

    // Control with joystick
    tank.addJoystick(joystick);

    // Run client on background thread, catching any exceptions for rethrowing
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C
    client.runInBackground();

    // Object to run object circumnavigator
    auto runner = createRunner(tank, viconObject);
    do {
        // Check for exceptions on background thread
        catcher.check();

        // Poll for joystick events
        joystick.update();
    } while (runner->update());

    return EXIT_SUCCESS;
}
