#include "hid/joystick_glfw_keyboard.h"

// GLFW
#include <GLFW/glfw3.h>

namespace BoBRobotics {
namespace HID {

//------------------------------------------------------------------------
// BoBRobotics::HID::JoystickGLFWKeyboard
//------------------------------------------------------------------------
JoystickGLFWKeyboard::JoystickGLFWKeyboard(GLFWwindow *window)
  : JoystickBase(0.0f), m_Window(window)
{

}
//------------------------------------------------------------------------
bool JoystickGLFWKeyboard::updateState()
{
    // Use WSAD for left stick axes
    setAxisStateKeys(GLFW_KEY_W, GLFW_KEY_S, JAxis::LeftStickVertical);
    setAxisStateKeys(GLFW_KEY_A, GLFW_KEY_D, JAxis::LeftStickHorizontal);

    // Use cursor for right stick axes
    setAxisStateKeys(GLFW_KEY_UP, GLFW_KEY_DOWN, JAxis::RightStickVertical);
    setAxisStateKeys(GLFW_KEY_LEFT, GLFW_KEY_RIGHT, JAxis::RightStickHorizontal);

    // Use page up and down for right trigger
    setAxisStateKeys(GLFW_KEY_PAGE_UP, GLFW_KEY_PAGE_DOWN, JAxis::RightTrigger);

    setButtonStateKey(GLFW_KEY_1, JButton::A);
    setButtonStateKey(GLFW_KEY_2, JButton::B);
    setButtonStateKey(GLFW_KEY_3, JButton::X);
    setButtonStateKey(GLFW_KEY_4, JButton::Y);

    return true;
}
//------------------------------------------------------------------------
void JoystickGLFWKeyboard::setAxisStateKeys(int negativeKey, int positiveKey, JAxis axis)
{
    if(glfwGetKey(m_Window, negativeKey) == GLFW_PRESS) {
        setState(axis, -1.0f, false);
    }
    else if(glfwGetKey(m_Window, positiveKey) == GLFW_PRESS) {
        setState(axis, 1.0f, false);
    }
    else if((glfwGetKey(m_Window, negativeKey) == GLFW_RELEASE) || (glfwGetKey(m_Window, positiveKey) == GLFW_RELEASE)) {
        setState(axis, 0.0f, false);
    }
}
//------------------------------------------------------------------------
void JoystickGLFWKeyboard::setButtonStateKey(int key, JButton button)
{
    if(glfwGetKey(m_Window, key) == GLFW_PRESS) {
        setPressed(button, false);
    }
    else if(glfwGetKey(m_Window, key) == GLFW_RELEASE) {
        setReleased(button, false);
    }
}
} // HID
} // BoBRobotics
