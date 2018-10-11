// Windows headers
#include "os/windows_include.h"

// If we're compiling on Windows, we know we don't need I2C
#ifdef _WIN32
#define NO_I2C_ROBOT
#endif

// BoB robotics includes
#include "net/server.h"
#include "os/net.h"
#include "robots/tank.h"
#include "video/netsink.h"
#include "video/panoramic.h"

#ifndef NO_I2C_ROBOT
#include "robots/norbot.h"
#endif

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace std::literals;
using namespace BoBRobotics;

int
main()
{
    try {
        auto cam = Video::getPanoramicCamera();

        // Enable networking on Windows
        OS::Net::WindowsNetworking net;

        // Listen for incoming connection on default port
        Net::Server server;

        // Stream camera synchronously over network
        Video::NetSink netSink(server, cam->getOutputSize(), cam->getCameraName());

#ifdef NO_I2C_ROBOT
        // Output motor commands to terminal
        Robots::Tank tank;
#else
        // Use Arduino robot
        Robots::Norbot tank;
#endif

        // Read motor commands from network
        tank.readFromNetwork(server);

        // Run server in background,, catching any exceptions for rethrowing
        BackgroundException::enableCatching();
        server.runInBackground();

        // Send frames over network
        cv::Mat frame;
        while (true) {
            // Rethrow any exceptions caught on background thread
            BackgroundException::check();

            // If there's a new frame, send it, else sleep
            if (cam->readFrame(frame)) {
                netSink.sendFrame(frame);
            } else {
                std::this_thread::sleep_for(25ms);
            }
        }
    } catch (Net::SocketClosingError &) {
        // The connection was closed on purpose: do nothing
        std::cout << "Connection closed" << std::endl;
    } catch (std::exception &e) {
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
        return 1;
    }
}
