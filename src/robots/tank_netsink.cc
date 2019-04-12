// BoB robotics includes
#include "common/assert.h"
#include "common/logging.h"
#include "common/stopwatch.h"
#include "robots/tank_netsink.h"

namespace BoBRobotics {
namespace Robots {

TankNetSink::TankNetSink(Net::Connection &connection)
    : m_Connection(connection)
{
    connection.setCommandHandler("TNK_PARAMS", [this](Net::Connection &, const Net::Command &command) {
        if (command.size() != 5) {
            throw Net::BadCommandError();
        }

        m_TurnSpeed = radians_per_second_t(stod(command[1]));
        m_ForwardSpeed = meters_per_second_t(stod(command[2]));
        m_AxisLength = millimeter_t(stod(command[3]));
        Tank::setMaximumSpeedProportion(stof(command[4]));
    });

    /*
        * If we start running the connection on a separate thread before this,
        * there's a chance the TNK_PARAMS command won't be caught.
        */
    BOB_ASSERT(!connection.isRunning());

    // Wait for command
    while (connection.readNextCommand() != "TNK_PARAMS")
        ;
}

TankNetSink::~TankNetSink()
{
    try {
        stopMoving();
    } catch (OS::Net::NetworkError &e) {
        LOG_WARNING << "Caught exception while trying to send command: "
                    << e.what();
    } catch (Net::SocketClosedError &) {
        // Socket has already been cleanly closed
    }

    // Stop listening for incoming commands
    m_Connection.setCommandHandler("TNK_PARAMS", nullptr);

    stopReadingFromNetwork();
}

void TankNetSink::setMaximumSpeedProportion(float value)
{
    if (value != getMaximumSpeedProportion()) {
        Tank::setMaximumSpeedProportion(value);

        m_Connection.getSocketWriter().send("TNK_MAX " + std::to_string(value) + "\n");
    }
}

//! Motor command: send TNK command over TCP
void TankNetSink::tank(float left, float right)
{
    BOB_ASSERT(left >= -1.f && left <= 1.f);
    BOB_ASSERT(right >= -1.f && right <= 1.f);

    // don't send a command if it's the same as the last one
    if (left == m_OldLeft && right == m_OldRight) {
        return;
    }

    // time how long it takes to send command
    Stopwatch netTimer;
    netTimer.start();

    // send steering command
    m_Connection.getSocketWriter().send("TNK " + std::to_string(left) + " " +
                                        std::to_string(right) + "\n");

    // print warning if steering command was slow to send
    using namespace std::literals;
    const auto duration = netTimer.elapsed();
    LOG_WARNING_IF(duration > 100ms) << "Network is slow ("
                                        << static_cast<units::time::millisecond_t>(duration)
                                        << " to send motor command)";

    // store current left/right values to compare next time
    m_OldLeft = left;
    m_OldRight = right;
}

units::length::millimeter_t TankNetSink::getRobotWidth() const
{
    if (std::isnan(m_AxisLength.value())) {
        return Tank::getRobotWidth();
    } else {
        return m_AxisLength;
    }
}

units::velocity::meters_per_second_t TankNetSink::getAbsoluteMaximumSpeed() const
{
    if (std::isnan(m_ForwardSpeed.value())) {
        return Tank::getAbsoluteMaximumSpeed();
    } else {
        return m_ForwardSpeed;
    }
}

units::angular_velocity::radians_per_second_t TankNetSink::getAbsoluteMaximumTurnSpeed() const
{
    if (std::isnan(m_TurnSpeed.value())) {
        return Tank::getAbsoluteMaximumTurnSpeed();
    } else {
        return m_TurnSpeed;
    }
}

} // Robots
} // BoBRobotics
