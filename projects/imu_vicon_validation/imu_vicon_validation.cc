// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/bn055_imu.h"
#include "common/circstat.h"
#include "common/macros.h"
#include "common/stopwatch.h"
#include "hid/joystick.h"
#include "hid/robot_control.h"
#include "robots/omni2d/mecanum.h"
#include "vicon/udp.h"

// Third-party includes
#include "third_party/path.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>

using namespace BoBRobotics;
using namespace std::literals;

int bobMain(int argc, char **argv)
{
    constexpr auto updateInterval = 100ms;
    using LengthUnit = units::length::millimeter_t;
    using AngleUnit = units::angle::degree_t;
    using TimeUnit = units::time::millisecond_t;
    const std::string objectName = "mecanum2";
    constexpr auto viconTimeout = 1s;

    BOB_ASSERT(argc <= 2);
    filesystem::path csvFilePath = argc == 2 ? argv[1] : "data.csv";
    BOB_ASSERT(!csvFilePath.exists());

    Robots::Omni2D::Mecanum robot;
    HID::Joystick joystick;
    BN055 imu;
    Vicon::UDPClient<> vicon(51001);
    auto viconObject = vicon.getObjectReference(objectName, viconTimeout);
    std::ofstream ofs(csvFilePath.str());
    ofs.exceptions(std::ios::badbit);
    ofs << "Time,ViconFrame,X,Y,Z,ViconAng1,ViconAng2,ViconAng3,ImuAng1,ImuAng2,ImuAng3\n";

    Stopwatch runTimer;
    BackgroundExceptionCatcher catcher;
    catcher.trapSignals();

    LOGI << "Running";
    runTimer.start();
    while (!joystick.isPressed(HID::JButton::B)) {
        catcher.check(); // Check for Ctrl+C

        const TimeUnit time = runTimer.elapsed();
        const auto objectData = viconObject.getData();
        const Pose3<LengthUnit, AngleUnit> pose = objectData.getPose();
        std::array<AngleUnit, 3> imuAngles = imu.getEulerAngles();
        for (auto &ang : imuAngles) {
            ang = normaliseAngle180(ang);
        }

        ofs << time.value() << "," << objectData.getFrameNumber() << ","
            << pose.x().value() << "," << pose.y().value() << "," << pose.z().value() << ","
            << pose.yaw().value() << "," << pose.pitch().value() << "," << pose.roll().value() << ","
            << imuAngles[0].value() << "," << imuAngles[1].value() << "," << imuAngles[2].value()
            << "\n";

        if (joystick.update()) {
            HID::drive(robot, joystick);
        }

        std::this_thread::sleep_for(updateInterval - (runTimer.elapsed() % 100ms));
    }

    return EXIT_SUCCESS;
}
