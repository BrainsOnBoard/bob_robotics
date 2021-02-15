#ifdef __linux__
// BoB robotics includes
#include "robots/mecanum.h"

// Standard C includes
#include <cmath>
#include <cstdint>

// Standard C++ includes
#include <algorithm>
#include <vector>


namespace BoBRobotics {
namespace Robots {
Mecanum::Mecanum(bool alternativeWiring, const char *devicePath)
  : m_AlternativeWiring(alternativeWiring)
  , m_Serial(devicePath)
{
    auto tty = m_Serial.getAttributes();

    tty.c_cflag |= B9600;            /* set baud rate */
    tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      /* 8-bit characters */
    tty.c_cflag &= ~PARENB;  /* no parity bit */
    tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;  // set to 0 for non-blocking
    tty.c_cc[VTIME] = 5; // set timeout to 0.5 secs

    m_Serial.setAttributes(tty);
}

Mecanum::~Mecanum()
{
    stopReadingFromNetwork();
    stopMoving();
}
//----------------------------------------------------------------------------
// Omni2D virtuals
//----------------------------------------------------------------------------
void
Mecanum::omni2D(float forward, float sideways, float turn)
{
    // Cache left and right
    setWheelSpeed(forward, sideways, turn);

    // resolve to motor speeds
    const float m1 = m_AlternativeWiring ? (-sideways + forward - turn) : (+sideways - forward - turn);
    const float m2 = m_AlternativeWiring ? (+sideways + forward + turn) : (+sideways + forward + turn);
    const float m3 = m_AlternativeWiring ? (+sideways + forward - turn) : (-sideways + forward - turn);
    const float m4 = m_AlternativeWiring ? (-sideways + forward + turn) : (-sideways - forward + turn);

    driveMotors(m1, m2, m3, m4);

}
//----------------------------------------------------------------------------
void Mecanum::driveMotors(float m1, float m2, float m3, float m4)
{
    // clamp values to be between -1 and 1 after resolving
    const auto cap = [](float &val) { val = std::min(1.f, std::max(val, -1.f)); };
    cap(m1);
    cap(m2);
    cap(m3);
    cap(m4);

    // **NOTE** 254s are because 255 is line end character
    uint8_t buffer[9] = { m1 > 0, m2 > 0, m3 > 0, m4 > 0, (uint8_t)(fabs(m1) * 254.0f), (uint8_t)(fabs(m2) * 254.0f), (uint8_t)(fabs(m3) * 254.0f), (uint8_t)(fabs(m4) * 254.0f), 255 };

    // Send buffer
    write(buffer);
}
} // Robots
} // BoBRobotics
#endif	// __linux__
