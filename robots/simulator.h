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
#include <vector>

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

class Simulator
  : public SimulatedTank<units::length::millimeter_t, units::angle::degree_t> {
private:
    using millimeter_t = units::length::millimeter_t;
    using degree_t = units::angle::degree_t;
    using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using second_t = units::time::second_t;

    // Agent's forward velocity and turning speed
    static constexpr meters_per_second_t Velocity = 1_mps;
    static constexpr degrees_per_second_t TurnSpeed = 90_deg_per_s;

    // Scaling factor
    static constexpr millimeter_t MMPerPixel = 4_mm;

    // Window size
    static constexpr int WindowWidth = 800;
    static constexpr int WindowHeight = 600;

    meters_per_second_t m_v;  // velocity v (translational velocity)
    degrees_per_second_t m_w; // velocity w (rotational velocity)

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

        // set current velocities
        m_v = v;
        m_w = w;

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

    //! draws the rectangle at the desired coordinates
    void drawRectangleAtCoordinates(const SDL_Rect &rectangle)
    {
        SDL_SetRenderDrawColor(m_renderer, 0, 0, 255, 255);
        SDL_RenderFillRect(m_renderer, &rectangle);
    }

public:
    Simulator(const millimeter_t carWidth = 16.4_cm)
      : SimulatedTank(Velocity, carWidth)
    {
        SDL_Init(SDL_INIT_VIDEO);

        m_window = SDL_CreateWindow("Wheeled-robot simulator",
            SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WindowWidth, WindowHeight, 0);

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
        m_robot_rect.x = WindowWidth / 2;
        m_robot_rect.y = WindowHeight / 2;
        const double widthPx = carWidth / MMPerPixel;
        m_robot_rect.h = static_cast<int>(widthPx);
        m_robot_rect.w = static_cast<int>((444.0 / 208.0) * widthPx);

        // first goal is set to the middle of the window
        m_mouse_click_position[0] = m_robot_rect.x;
        m_mouse_click_position[1] = m_robot_rect.y;

        SimulatedTank::setPose({ m_robot_rect.x * MMPerPixel,
                                 m_robot_rect.y * MMPerPixel, 10_deg });
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
        m_robot_rect.x = pose.x / MMPerPixel;
        m_robot_rect.y = pose.y / MMPerPixel;
    }

    //! returns true if we did quit the simulator's gui
    bool didQuit() const
    {
        return m_quit;
    }

    //! suimulates a step of the simulation with the provided velocities
    bool simulationStep(meters_per_second_t v,
                        degrees_per_second_t w,
                        second_t delta_time)
    {
        bool ret = false;

        // Clear the entire screen to our selected color.
        SDL_RenderClear(m_renderer);

        // getting events
        SDL_PollEvent(&m_event);
        switch (m_event.type) {
        case SDL_QUIT:
            m_quit = true;
            return false;

        case SDL_KEYDOWN:
            switch (m_event.key.keysym.sym) {
            case SDLK_LEFT:
                w = -TurnSpeed;
                break;
            case SDLK_RIGHT:
                w = TurnSpeed;
                break;
            case SDLK_UP:
                v = Velocity;
                w = 0_deg_per_s;
                break;
            case SDLK_DOWN:
                v = -Velocity;
                w = 0_deg_per_s;
                break;
            case SDLK_SPACE:
                ret = true;
                break;
            }
            break;

        case SDL_MOUSEBUTTONDOWN:
            // If the left button was pressed.
            if (m_event.button.button == SDL_BUTTON_LEFT) {
                SDL_GetMouseState(&m_mouse_click_position[0], &m_mouse_click_position[1]);
            }
            break;

        default:
            break;
        }

        updatePose(v, w, delta_time);

        // draw a rectangle at the goal position
        drawRectangleAtCoordinates({ m_mouse_click_position[0], m_mouse_click_position[1], 5, 5 });

        // Select the color for drawing.
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 255);

        // render texture with rotation
        SDL_RenderCopyEx(m_renderer, m_texture, nullptr, &m_robot_rect, getPose().angle.value(), nullptr, SDL_FLIP_NONE);

        SDL_RenderPresent(m_renderer);

        return ret;
    }

    //! gets the position of the latest mouse click relative to the window
    Vector2<millimeter_t> getMouseClickLocation() const
    {
        return { m_mouse_click_position[0] * MMPerPixel,
                 m_mouse_click_position[1] * MMPerPixel };
    }

    //! changes the pixel value to millimeter
    static void changePixelToMM(const float x_pos,
                                const float y_pos,
                                millimeter_t &x_mm,
                                millimeter_t &y_mm)
    {
        x_mm = x_pos * MMPerPixel;
        y_mm = y_pos * MMPerPixel;
    }
}; // Simulator

#ifndef NO_HEADER_DEFINITIONS
constexpr units::velocity::meters_per_second_t Simulator::Velocity;
constexpr units::angular_velocity::degrees_per_second_t Simulator::TurnSpeed;
constexpr units::length::millimeter_t Simulator::MMPerPixel;
constexpr int Simulator::WindowWidth, Simulator::WindowHeight;
#endif
} // Robots
} // BoBRobotics
