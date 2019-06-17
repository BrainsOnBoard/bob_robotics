// BoB robotics includes
#include "robots/tank_netsink.h"
#include "common/logging.h"
#include "common/macros.h"
#include "common/stopwatch.h"

namespace BoBRobotics {
namespace Robots {

BundledTankNetSink::BundledTankNetSink()
  : TankNetSinkBase<Net::Client>()
{
    // Run client on background thread
    getConnection().runInBackground();
}

template<class ConnectionType>
TankNetSinkBase<ConnectionType>::~TankNetSinkBase()
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

template<class ConnectionType>
void
TankNetSinkBase<ConnectionType>::setMaximumSpeedProportion(float value)
{
    if (value != getMaximumSpeedProportion()) {
        Tank::setMaximumSpeedProportion(value);

        m_Connection.getSocketWriter().send("TNK_MAX " + std::to_string(value) + "\n");
    }
}

//! Motor command: send TNK command over TCP
template<class ConnectionType>
void
TankNetSinkBase<ConnectionType>::tank(float left, float right)
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

template<class ConnectionType>
units::length::millimeter_t
TankNetSinkBase<ConnectionType>::getRobotWidth() const
{
    if (std::isnan(m_AxisLength.value())) {
        return Tank::getRobotWidth();
    } else {
        return m_AxisLength;
    }
}

template<class ConnectionType>
units::velocity::meters_per_second_t
TankNetSinkBase<ConnectionType>::getAbsoluteMaximumSpeed() const
{
    if (std::isnan(m_ForwardSpeed.value())) {
        return Tank::getAbsoluteMaximumSpeed();
    } else {
        return m_ForwardSpeed;
    }
}

template<class ConnectionType>
units::angular_velocity::radians_per_second_t
TankNetSinkBase<ConnectionType>::getAbsoluteMaximumTurnSpeed() const
{
    if (std::isnan(m_TurnSpeed.value())) {
        return Tank::getAbsoluteMaximumTurnSpeed();
    } else {
        return m_TurnSpeed;
    }
}

} // Robots
} // BoBRobotics
