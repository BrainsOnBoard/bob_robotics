// Windows headers
#include "os/windows_include.h"

// If we're compiling on Windows, we know we don't need I2C
#ifdef _WIN32
#define NO_I2C_ROBOT
#endif

#include "navigation_common.h"

// BoB robotics includes
#include "common/main.h"
#include "robots/tank.h"
#include "vicon/udp.h"
#include "video/panoramic.h"
#include "video/unwrapped_input.h"
#ifdef NO_I2C_ROBOT
#include "net/client.h"
#include "robots/tank_netsink.h"
#include "video/netsource.h"
#else
#include "robots/norbot.h"
#endif

using namespace BoBRobotics;
using namespace units::literals;

auto
loadObjects(const std::string &objectsPath)
{
    std::vector<ObjectType> objects;

    std::cout << "Loading object positions from " << objectsPath << "..." << std::endl;
    cv::FileStorage fs(objectsPath, cv::FileStorage::READ);
    std::vector<double> vertex(2);
    for (auto objectNode : fs["objects"]) {
        objects.emplace_back();
        for (auto vertexNode : objectNode) {
            vertexNode >> vertex;
            objects.back().first.emplace_back(vertex[0]);
            objects.back().second.emplace_back(vertex[1]);
        }
    }

    return objects;
}

int
bob_main(int argc, char **argv)
{
    const float forwardSpeed = 1.f;
    const cv::Size unwrapResolution{ 360, 75 };

    const auto objects = loadObjects("../../tools/vicon_arena_constructor/objects.yaml");

    Vicon::UDPClient<> vicon(51001);

#ifdef NO_I2C_ROBOT
    std::unique_ptr<Net::Client> client;
    std::unique_ptr<Robots::Tank> tank;
    std::unique_ptr<Video::Input> cam;
    if ((argc > 1 && strcmp(argv[1], "--robot") == 0)) {
        cam = Video::getPanoramicCamera();
        tank = std::make_unique<Robots::Tank>();
    } else {
        std::string robotIP;
        if (argc == 2) {
            // Get robot IP from command-line argument
            robotIP = argv[1];
        } else {
            // Get robot IP from terminal
            std::cout << "Robot IP [127.0.0.1]: ";
            std::getline(std::cin, robotIP);
            if (robotIP.empty()) {
                robotIP = "127.0.0.1";
            }
        }

        // Make connection to robot on default port
        client = std::make_unique<Net::Client>(robotIP);

        // Output motor commands to network
        tank = std::make_unique<Robots::TankNetSink>(*client);

        // Read video stream from network
        cam = std::make_unique<Video::NetSource>(*client);

        client->runInBackground();
    }
#else
    // Use Arduino robot
    auto tank = std::make_unique<Robots::Norbot>();
    auto cam = Video::getPanoramicCamera();
#endif

    // Unwrap camera frames
    Video::UnwrappedInput camUnwrapped(*cam, unwrapResolution);

    // Wait for Vicon system to start
    while (vicon.getNumObjects() == 0) {
        std::this_thread::sleep_for(1s);
        std::cout << "Waiting for object" << std::endl;
    }

    auto viconObject = vicon.getObject(0);
    const units::length::millimeter_t lim = 3000_mm;
    runNavigation(*tank, viconObject, forwardSpeed, camUnwrapped, -lim, lim, -lim, lim, objects);

    return EXIT_SUCCESS;
}
