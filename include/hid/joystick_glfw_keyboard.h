#pragma once

// BoB robotics includes
#include "joystick.h"

// Forward declarations
class GLFWwindow;

namespace BoBRobotics {
namespace HID {

/*!
 * \brief Class for reading from joysticks on Linux.
 *
 * *NOTE*: This class should not be used directly; see example in joystick_test.
 */
class JoystickGLFWKeyboard : public JoystickBase<JAxis, JButton>
{
public:
    //! Open default joystick device with (optionally) specified dead zone
    JoystickGLFWKeyboard(GLFWwindow *window);

protected:
    //------------------------------------------------------------------------
    // JoystickBase virtuals
    //------------------------------------------------------------------------
    virtual bool updateState() override;

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    void setAxisStateKeys(int negativeKey, int positiveKey, JAxis axis);
    void setButtonStateKey(int key, JButton button);

    GLFWwindow *m_Window;
}; // JoystickLinux
} // HID
} // BoBRobotics
