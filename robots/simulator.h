#pragma once

// BoB robotics includes
#include "../common/pose.h"
#include "../robots/simulated_tank.h"

// Third-party includes
#include "../third_party/units.h"

// SDL
#include <SDL2/SDL.h>

// Standard C includes
#include <cmath>
#include <cstdlib>

// Standard C++ includes
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

class Simulator
  : public SimulatedTank<units::length::millimeter_t, units::angle::degree_t>
{
    using meter_t = units::length::meter_t;
    using millimeter_t = units::length::millimeter_t;
    using degree_t = units::angle::degree_t;
    using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using second_t = units::time::second_t;

public:
    Simulator(const millimeter_t screenHeight = 3.2_m, const meters_per_second_t speed = 0.05_mps, const millimeter_t carWidth = 16.4_cm)
      : SimulatedTank(speed, carWidth)
      , m_MMPerPixel(screenHeight / WindowHeight)
    {
        SDL_Init(SDL_INIT_VIDEO);

        m_window = SDL_CreateWindow("Wheeled-robot simulator",
                                    SDL_WINDOWPOS_UNDEFINED,
                                    SDL_WINDOWPOS_UNDEFINED,
                                    WindowWidth,
                                    WindowHeight,
                                    0);

        m_renderer = SDL_CreateRenderer(m_window, -1, 0);

        const std::string imagePath = std::string(std::getenv("BOB_ROBOTICS_PATH")) + "/robots/car.bmp";
        SDL_Surface *image = SDL_LoadBMP(imagePath.c_str());
        BOB_ASSERT(image != nullptr); // Check file exists

        // Turn bitmap into texture
        m_texture = SDL_CreateTextureFromSurface(m_renderer, image);
        SDL_FreeSurface(image); // We're done with bitmap

        // Select the color for drawing.
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 255);

        // Clear the entire screen to our selected color.
        SDL_RenderClear(m_renderer);

        // initial position and size of the robot car
        const double widthPx = carWidth / m_MMPerPixel;
        m_robot_rect.h = static_cast<int>(widthPx);
        m_robot_rect.w = static_cast<int>((444.0 / 208.0) * widthPx);

        // first goal is set to the middle of the window
        m_mouse_click_position[0] = WindowWidth / 2;
        m_mouse_click_position[1] = WindowHeight / 2;

        setRobotPosition(0_m, 0_m);
    }

    virtual ~Simulator() override
    {
        // freeing up resources
        SDL_DestroyTexture(m_texture);
        SDL_DestroyRenderer(m_renderer);
        SDL_DestroyWindow(m_window);
        SDL_Quit();
    }

    //! sets the current pose of the robot
    void setPose(const Pose2<millimeter_t, degree_t> &pose)
    {
        SimulatedTank::setPose(pose);

        // Update agent's position in pixels
        setRobotPosition(pose.x, pose.y);
    }

    //! returns true if we did quit the simulator's gui
    bool didQuit() const
    {
        return m_quit;
    }

    SDL_Keycode simulationStep()
    {
        const auto key = pollEvents();
        if (key.second) {
            switch (key.first) {
            case SDLK_LEFT:
                tank(-0.5f, 0.5f);
                break;
            case SDLK_RIGHT:
                tank(0.5f, -0.5f);
                break;
            case SDLK_UP:
                tank(1.f, 1.f);
                break;
            case SDLK_DOWN:
                tank(-1.f, -1.f);
                break;
            }
        } else {
            switch (key.first) {
            case SDLK_LEFT:
            case SDLK_RIGHT:
            case SDLK_UP:
            case SDLK_DOWN:
                stopMoving();
                break;
            }
        }

        const auto &pose = getPose();

        // Update agent's position in pixels
        setRobotPosition(pose.x, pose.y);

        draw();

        return key.second ? key.first : 0;
    }

    //! suimulates a step of the simulation with the provided velocities
    SDL_Keycode simulationStep(meters_per_second_t v,
                               degrees_per_second_t w,
                               second_t delta_time)
    {
        const auto key = pollEvents();
        if (key.second) {
            switch (key.first) {
            case SDLK_LEFT:
                w = getMaximumTurnSpeed();
                break;
            case SDLK_RIGHT:
                w = -getMaximumTurnSpeed();
                break;
            case SDLK_UP:
                v = getMaximumSpeed();
                w = 0_deg_per_s;
                break;
            case SDLK_DOWN:
                v = -getMaximumSpeed();
                w = 0_deg_per_s;
                break;
            }
        }

        updatePose(v, w, delta_time);

        draw();

        return key.second ? key.first : 0;
    }

    //! gets the position of the latest mouse click relative to the window
    Vector2<millimeter_t> getMouseClickLocation() const
    {
        Vector2<millimeter_t> out;
        pixelToMM(m_mouse_click_position[0], m_mouse_click_position[1], out[0], out[1]);
        return out;
    }

private:
    // Window size
    static constexpr int WindowWidth = 800;
    static constexpr int WindowHeight = 600;

    millimeter_t m_MMPerPixel; // Scaling factor

    Vector2<int> m_mouse_click_position;

    bool m_quit = false;
    SDL_Window *m_window;
    SDL_Event m_event;
    SDL_Renderer *m_renderer;
    SDL_Texture *m_texture;
    SDL_Rect m_robot_rect;

    void updatePose(const meters_per_second_t v,
                    const degrees_per_second_t w,
                    const second_t dt)
    {
        using namespace units::length;
        using namespace units::math;

        auto pose = getPose();
        if (w == 0_deg_per_s) {
            const meter_t r = v * dt;
            pose.x += r * cos(pose.angle);
            pose.y += r * sin(pose.angle);
        } else {
            // v = wr, but the units lib gives a mismatched units error for it
            const units::angular_velocity::radians_per_second_t w_rad = w;
            const meter_t r{ (v / w_rad).value() };
            const degree_t newAngle = pose.angle + w * dt;
            pose.x += -r * sin(pose.angle) + r * sin(newAngle);
            pose.y += r * cos(pose.angle) - r * cos(newAngle);
            pose.angle = newAngle;
        }
        setPose(pose);
    }

    std::pair<SDL_Keycode, bool> pollEvents()
    {
        // Clear the entire screen to our selected color.
        SDL_RenderClear(m_renderer);

        // getting events
        SDL_PollEvent(&m_event);
        switch (m_event.type) {
        case SDL_QUIT:
            m_quit = true;
            break;
        case SDL_KEYDOWN:
            return std::make_pair(m_event.key.keysym.sym, true);
        case SDL_KEYUP:
            return std::make_pair(m_event.key.keysym.sym, false);
        case SDL_MOUSEBUTTONDOWN:
            // If the left button was pressed.
            if (m_event.button.button == SDL_BUTTON_LEFT) {
                SDL_GetMouseState(&m_mouse_click_position[0], &m_mouse_click_position[1]);
            }
            break;
        default:
            break;
        }

        return std::make_pair(0, false);
    }

    //! draws the rectangle at the desired coordinates
    void drawRectangleAtCoordinates(const SDL_Rect &rectangle)
    {
        SDL_SetRenderDrawColor(m_renderer, 0, 0, 255, 255);
        SDL_RenderFillRect(m_renderer, &rectangle);
    }

    void draw()
    {
        // draw a rectangle at the goal position
        drawRectangleAtCoordinates({ m_mouse_click_position[0], m_mouse_click_position[1], 5, 5 });

        // Select the color for drawing.
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 255);

        // render texture with rotation
        SDL_RenderCopyEx(m_renderer, m_texture, nullptr, &m_robot_rect,
                         -getPose().angle.value(), nullptr, SDL_FLIP_NONE);

        SDL_RenderPresent(m_renderer);
    }

    void pixelToMM(const int x, const int y, millimeter_t &xMM, millimeter_t &yMM) const
    {
        xMM = (x - (WindowWidth / 2)) * m_MMPerPixel;
        yMM = -(y - (WindowHeight / 2)) * m_MMPerPixel;
    }

    void mmToPixel(const millimeter_t x, const millimeter_t y, int &xPixel, int &yPixel) const
    {
        xPixel = static_cast<int>(x / m_MMPerPixel) + (WindowWidth / 2);
        yPixel = -static_cast<int>(y / m_MMPerPixel) + (WindowHeight / 2);
    }

    void setRobotPosition(const millimeter_t x, const millimeter_t y)
    {
        mmToPixel(x, y, m_robot_rect.x, m_robot_rect.y);
        m_robot_rect.x -= m_robot_rect.w / 2;
        m_robot_rect.y -= m_robot_rect.h / 2;
    }
}; // Simulator

#ifndef NO_HEADER_DEFINITIONS
constexpr int Simulator::WindowWidth, Simulator::WindowHeight;
#endif
} // Robots
} // BoBRobotics
