#ifdef __linux__
// BoB robotics includes
#include "common/macros.h"
#include "robots/omni2d/mecanum.h"

// Standard C includes
#include <cmath>
#include <cstdint>

// Standard C++ includes
#include <algorithm>
#include <vector>


namespace BoBRobotics {
namespace Robots {
namespace Omni2D {

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
    stopMoving();
}
//----------------------------------------------------------------------------
// Omni2DBase virtuals
//----------------------------------------------------------------------------
void Mecanum::driveMotors(float m1, float m2, float m3, float m4)
{
    BOB_ASSERT(m1 >= -1.0f && m1 <= 1.0f);
    BOB_ASSERT(m2 >= -1.0f && m2 <= 1.0f);
    BOB_ASSERT(m3 >= -1.0f && m3 <= 1.0f);
    BOB_ASSERT(m4 >= -1.0f && m4 <= 1.0f);

    // **NOTE** 254s are because 255 is line end character
    uint8_t buffer[9] = { m1 > 0, m2 > 0, m3 > 0, m4 > 0, (uint8_t)(fabs(m1) * 254.0f), (uint8_t)(fabs(m2) * 254.0f), (uint8_t)(fabs(m3) * 254.0f), (uint8_t)(fabs(m4) * 254.0f), 255 };

    // Send buffer
    write(buffer);
}
//----------------------------------------------------------------------------
void
Mecanum::omni2DInternal(float forward, float sideways, float turn)
{
    // resolve to motor speeds
    const float m1 = m_AlternativeWiring ? (-sideways + forward - turn) : (+sideways - forward - turn);
    const float m2 = +sideways + forward + turn;
    const float m3 = m_AlternativeWiring ? (+sideways + forward - turn) : (-sideways + forward - turn);
    const float m4 = m_AlternativeWiring ? (-sideways + forward + turn) : (-sideways - forward + turn);

    // Cap before passing to driveMotors as valid forward, sideways and
    // turn values can still result in invalid m1, m2, m3 and m4
    const auto cap = [](float val) { return std::min(1.0f, std::max(val, -1.0f)); };
    driveMotors(cap(m1), cap(m2), cap(m3), cap(m4));
}
} // Omni2D
} // Robots
} // BoBRobotics
#endif	// __linux__
