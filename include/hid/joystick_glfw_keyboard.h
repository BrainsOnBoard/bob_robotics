#pragma once

// BoB robotics includes
#include "joystick.h"

// Forward declarations
class GLFWwindow;

namespace BoBRobotics {
namespace HID {

//------------------------------------------------------------------------
// BoBRobotics::HID::JoystickGLFWKeyboard
//------------------------------------------------------------------------
/*!
 * \brief Class for using keyboard (via GLFW) as a joystick
 */
class JoystickGLFWKeyboard : public JoystickBase<JAxis, JButton>
{
public:
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

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    GLFWwindow *m_Window;
}; // JoystickLinux
} // HID
} // BoBRobotics
