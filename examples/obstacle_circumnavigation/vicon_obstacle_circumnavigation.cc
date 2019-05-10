#include "common.h"

// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/main.h"
#include "hid/joystick.h"
#include "net/client.h"
#include "robots/tank_netsink.h"
#include "vicon/udp.h"

int
bob_main(int, char **)
{
    HID::Joystick joystick;

    // Connect to Vicon system
    Vicon::UDPClient<> vicon(51001);
    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }
    auto viconObject = vicon.getObjectReference(0, 5s);

    // Make connection to robot on default port
    Net::Client client;

    // Transmit motor commands over network
    Robots::TankNetSink tank(client);

    // Control with joystick
    tank.addJoystick(joystick);

    // Run client on background thread, catching any exceptions for rethrowing
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C
    client.runInBackground();

    runObstacleCircumnavigation(tank, viconObject,
        // Extra functions to be called in main loop
        [&]()
        {
            // Check for exceptions on background thread
            catcher.check();

            // Poll for joystick events
            joystick.update();
        });

    return EXIT_SUCCESS;
}
