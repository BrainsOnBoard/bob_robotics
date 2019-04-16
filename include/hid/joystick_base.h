#pragma once

// BoB robotics includes
#include "common/threadable.h"

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <array>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace BoBRobotics {
namespace HID {
//! The current state of a joystick button
enum ButtonState
{
    StateDown       = (1 << 0), //!< Whether the button is being pressed down
    StatePressed    = (1 << 1), //!< Whether the button has been pressed since last update()
    StateReleased   = (1 << 2) //!< Whether the button has been released since last update()
};

/*!
 * \brief Generic joystick class for Xbox 360 controller, from which
 *        JoystickLinux and JoystickWindows classes inherit.
 *
 * The class provides the basic, platform-independent functionality required
 * by JoystickLinux and JoystickWindows.
 *
 * *NOTE*: This class should not be used directly; see example in examples/joystick.
 */
template<typename JAxis, typename JButton>
class JoystickBase : public Threadable
{
    /*!
     * \brief A delegate to handle joystick button presses
     *
     * @param button Which button was pressed/released
     * @param pressed Whether button was pressed/released
     * @return True if the function has handled the event, false otherwise
     */
    using ButtonHandler = std::function<bool(JButton button, bool pressed)>;

    /*!
     * \brief A delegate to handle joystick axis events (i.e. moving joysticks)
     *
     * @param axis Which axis was pressed/released
     * @param value Value from -1.0f to 1.0f representing new position of axis
     * @return True if the function has handled the event, false otherwise
     */
    using AxisHandler = std::function<bool(JAxis axis, float value)>;

protected:
    template<class T>
    static constexpr size_t toIndex(T value) { return static_cast<size_t>(value); }

    template<class T>
    static constexpr JAxis toAxis(T value) { return static_cast<JAxis>(value); }

    template<class T>
    static constexpr JButton toButton(T value) { return static_cast<JButton>(value); }

    static constexpr float int16_maxf() { return static_cast<float>(std::numeric_limits<int16_t>::max()); }
    static constexpr float int16_absminf() { return -static_cast<float>(std::numeric_limits<int16_t>::min()); }

public:
    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    /*!
     * \brief Try to read from the joystick
     *
     * @return True if one or more events were read from joystick
     */
    bool update();

    //! Add a function to handle joystick axis events
    void addHandler(AxisHandler handler);

    //! Add a function to handle joystick button events
    void addHandler(ButtonHandler handler);

    //! Get the current value for a specified joystick axis
    float getState(JAxis axis) const;

    /*!
     * \brief Get the current state for a specified joystick button
     *
     * \see ButtonState
     */
    unsigned char getState(JButton button) const;

    //! Whether button is currently being pressed
    bool isDown(JButton button) const;

    //! Whether button has been pressed since last update()
    bool isPressed(JButton button) const;

    //! Whether button has been released since last update()
    bool isReleased(JButton button) const;

    //! Get the name of a specified joystick axis
    static std::string getName(JAxis axis);

    //! Get the name of a specified joystick button
    static std::string getName(JButton button);

protected:
    JoystickBase(float deadZone = 0.0f);

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    virtual bool updateState() = 0;

    //------------------------------------------------------------------------
    // Threadable virtuals
    //------------------------------------------------------------------------
    virtual void runInternal() override;

    //------------------------------------------------------------------------
    // Protected methods
    //------------------------------------------------------------------------
    void setPressed(JButton button, bool isInitial);

    void setReleased(JButton button, bool isInitial);

    void setState(JButton button, uint8_t state, bool isInitial);

    void setState(JAxis axis, float value, bool isInitial);

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    float getDeadZonedState(JAxis axis, JAxis axisPerpendicular) const;

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    std::array<uint8_t, toIndex(JButton::LENGTH)> m_ButtonState;
    std::vector<ButtonHandler> m_ButtonHandlers;
    std::vector<AxisHandler> m_AxisHandlers;
    std::array<float, toIndex(JAxis::LENGTH)> m_AxisState;
    const float m_DeadZone;
}; // JoystickBase
} // HID
} // BoBRobotics
