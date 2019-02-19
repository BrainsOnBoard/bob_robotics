#include "common.h"

// BoB robotics includes
#include "net/client.h"
#include "robots/tank_netsink.h"
#include "vicon/udp.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

using namespace std::literals;

int
bob_main(int, char **)
{
    constexpr float kp = 2.f;
    constexpr float ki = 0.5f;
    constexpr float kd = 0.1f;

    HID::Joystick joystick(0.25f);

    Vicon::UDPClient<> vicon(51001);
    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }
    auto object = vicon.getObjectReference(0, 5_s);

    // Make connection to robot on default port
    Net::Client client;

    Robots::TankPID<Robots::TankNetSink> robot(kp, ki, kd, client);

    client.runInBackground();
    runWheelPID(joystick, robot, object);

    return EXIT_SUCCESS;
}
