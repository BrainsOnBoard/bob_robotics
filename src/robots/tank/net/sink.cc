// BoB robotics includes
#include "common/macros.h"
#include "common/stopwatch.h"
#include "robots/tank/net/sink.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <stdexcept>

namespace BoBRobotics {
namespace Robots {
namespace Tank {
namespace Net {

// Explicitly instantiate
template class SinkBase<BoBRobotics::Net::Connection &>;

BundledSink::BundledSink()
{
    // Run client on background thread
    getConnection().runInBackground();
}

template<class ConnectionType>
SinkBase<ConnectionType>::~SinkBase()
{
    try {
        this->stopMoving();
    } catch (OS::Net::NetworkError &e) {
        LOG_WARNING << "Caught exception while trying to send command: "
                    << e.what();
    } catch (BoBRobotics::Net::SocketClosedError &) {
        // Socket has already been cleanly closed
    }

    // Stop listening for incoming commands
    m_Connection.setCommandHandler("TNK_PARAMS", nullptr);
}

template<class ConnectionType>
units::length::millimeter_t
SinkBase<ConnectionType>::getRobotWidth() const
{
    if (std::isnan(m_AxisLength.value())) {
        throw std::runtime_error("No value given for robot width");
    }

    return m_AxisLength;
}

template<class ConnectionType>
units::velocity::meters_per_second_t
SinkBase<ConnectionType>::getMaximumSpeed() const
{
    if (std::isnan(m_ForwardSpeed.value())) {
        throw std::runtime_error("No value given for maximum speed");
    }

    return m_ForwardSpeed;
}

template<class ConnectionType>
units::angular_velocity::radians_per_second_t
SinkBase<ConnectionType>::getMaximumTurnSpeed() const
{
    if (std::isnan(m_TurnSpeed.value())) {
        throw std::runtime_error("No value given for maximum turn speed");
    }

    return m_TurnSpeed;
}

//! Motor command: send TNK command over TCP
template<class ConnectionType>
void
SinkBase<ConnectionType>::tankInternal(float left, float right)
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

} // Net
} // Tank
} // Robots
} // BoBRobotics
