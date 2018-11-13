#pragma once

// BoB robotics includes
#include "../common/pose.h"

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
class Simulator {
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
    degree_t m_angle;         // angle of the robot
    millimeter_t m_x;         // position 'x' of the robot on the screen
    millimeter_t m_y;         // position 'y' of the robot on the screen

    // first goal is set to the middle of the window
    Vector2<int> m_mouse_click_position{ WindowWidth / 2, WindowHeight / 2 };

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

        if (w == 0_deg_per_s) {
            const meter_t r = v * dt;
            setPose(m_x + r * cos(m_angle), m_y + r * sin(m_angle), m_angle);
        } else {
            // v = wr, but the units lib gives a mismatched units error for it
            const units::angular_velocity::radians_per_second_t w_rad = w;
            const meter_t r{ (v / w_rad).value() };
            const degree_t new_angle = m_angle + w * dt;
            const auto x = m_x + -r * sin(m_angle) + r * sin(new_angle);
            const auto y = m_y + r * cos(m_angle) - r * cos(new_angle);
            setPose(x, y, new_angle);
        }
    }

    //! draws the rectangle at the desired coordinates
    void drawRectangleAtCoordinates(const SDL_Rect &rectangle)
    {
        SDL_SetRenderDrawColor(m_renderer, 0, 0, 255, 255);
        SDL_RenderFillRect(m_renderer, &rectangle);
    }

public:
    Simulator()
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
        m_robot_rect = { WindowWidth / 2, WindowHeight / 2, 10, 13 };

        m_x = m_robot_rect.x * MMPerPixel;
        m_y = m_robot_rect.y * MMPerPixel;
        m_angle = 10_deg;
    }

    ~Simulator()
    {
        // freeing up resources
        SDL_DestroyTexture(m_texture);
        SDL_DestroyRenderer(m_renderer);
        SDL_DestroyWindow(m_window);
        SDL_Quit();
    }

    //! sets the current pose of the robot
    void setPose(const millimeter_t x, const millimeter_t y, const degree_t theta)
    {
        m_x = x;
        m_y = y;
        m_angle = theta;

        // Update agent's position in pixels
        m_robot_rect.x = m_x / MMPerPixel;
        m_robot_rect.y = m_y / MMPerPixel;
    }

    //! sets the robot's size in millimeter
    void setRobotSize(const millimeter_t height,
                      const millimeter_t width
                      )
    {
        m_robot_rect.h = height / MMPerPixel;
        m_robot_rect.w = width / MMPerPixel;
    }

    //! returns true if we did quit the simulator's gui
    bool didQuit() const
    {
        return m_quit;
    }

    //! suimulates a step of the simulation with the provided velocities
    bool simulationStep(meters_per_second_t v,
                        degrees_per_second_t w,
                        second_t delta_time
                        )
    {
        bool ret = false;

        // Clear the entire screen to our selected color.
        SDL_RenderClear(m_renderer);

        // getting events
        SDL_PollEvent(&m_event);
        switch (m_event.type) {
        case SDL_QUIT:
            m_quit = true;
            break;

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
        SDL_RenderCopyEx(m_renderer, m_texture, nullptr, &m_robot_rect, m_angle.value(), nullptr, SDL_FLIP_NONE);

        SDL_RenderPresent(m_renderer);

        return ret;
    }

    //! returns the current position of the robot
    Vector3<float> getCurrentPosition()
    {
        return { static_cast<float>(m_robot_rect.x), static_cast<float>(m_robot_rect.y), m_angle.value() };
    }

    //! gets the position of the latest mouse click relative to the window
    Vector2<int> getMouseClickLocation()
    {
        return m_mouse_click_position;
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
