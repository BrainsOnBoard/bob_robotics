// BoB robotics includes
#include "net/imu_netsource.h"

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace Net {
IMUNetSource::IMUNetSource(Net::Connection &connection)
  : m_Connection(connection)
{
    connection.setCommandHandler("IMU_ANG",
        [this](Net::Connection &, const Net::Command &command)
        {
            m_Heading = std::stod(command.at(1));
            m_Semaphore.notify();
        });
}

units::angle::radian_t
IMUNetSource::getHeading()
{
    m_Connection.getSocketWriter().send("IMU_REQ\n");
    m_Semaphore.wait();
    return units::angle::radian_t{ m_Heading };
}

} // Net
} // BoBRobotics