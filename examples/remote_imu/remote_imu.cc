// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/logging.h"
#include "common/main.h"
#include "net/client.h"
#include "net/imu_netsource.h"
#include "robots/tank_netsink.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace BoBRobotics;

int bob_main(int, char **)
{
    // Make connection to robot on default port
    Net::Client client;

    Robots::TankNetSink tank(client);

    // Read IMU over network
    Net::IMUNetSource imu(client);

    // Run client on background thread, catching any exceptions for rethrowing
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals(); // Catch Ctrl-C
    client.runInBackground();

    while (true) {
        catcher.check();
        LOGI << "Robot heading: " << imu.getYaw();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return EXIT_SUCCESS;
}