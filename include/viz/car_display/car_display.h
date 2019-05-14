#pragma once

// BoB robotics includes
#include "common/pose.h"

// Third-party includes
#include "third_party/units.h"

// SDL
#include <SDL2/SDL.h>

// Standard C++ includes
#include <array>
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Viz {
using namespace units::literals;

class CarDisplay
{
    using meter_t = units::length::meter_t;
    using millimeter_t = units::length::millimeter_t;
    using degree_t = units::angle::degree_t;
    using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using second_t = units::time::second_t;

public:
    CarDisplay(const millimeter_t screenHeight = 3.2_m, const millimeter_t carWidth = 16.4_cm);

    ~CarDisplay();

    //! Returns true if GUI is still running
    bool isOpen() const;

    std::pair<SDL_Keycode, bool> runGUI(const Pose2<millimeter_t, degree_t> &agentPose);

    Vector2<millimeter_t> getMouseClickPosition() const;

    std::vector<float> getMouseClickPixelPosition() const;

    //! draws the rectangle at the desired coordinates
    void drawRectangleAtCoordinates(const SDL_Rect &rectangle);

    //! return the current SDL_Renderer pointer if we want to draw additional graphics
    //! elements to the screen.
    SDL_Renderer *getRenderer();

    //! clears the screen to the selected color. Call this first, before draw()
    void clearScreen();

    //! changes pixel to millimeter
    void pixelToMM(const int x, const int y, millimeter_t &xMM, millimeter_t &yMM) const;

    //! change from millimeter to pixel
    void mmToPixel(const millimeter_t x, const millimeter_t y, int &xPixel, int &yPixel) const;

private:
    // Window size
    static constexpr int WindowWidth = 800;
    static constexpr int WindowHeight = 600;

    millimeter_t m_MMPerPixel; // Scaling factor

    std::array<int, 2> m_MouseClickPosition;

    bool m_IsOpen = true;
    SDL_Window *m_Window;
    SDL_Renderer *m_Renderer;
    SDL_Texture *m_Texture;
    SDL_Rect m_RobotRectangle;

    std::pair<SDL_Keycode, bool> pollEvents();

    void draw(const degree_t agentAngle);
    void setRobotPosition(const millimeter_t x, const millimeter_t y);
}; // CarDisplay
} // Viz
} // BoBRobotics
