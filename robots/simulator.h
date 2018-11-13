#pragma once

// Third-party includes
#include "../third_party/units.h"

// SDL
#include <SDL2/SDL.h>

// Standard C includes
#include <cstdlib>
#include <cmath>

// Standard C++ includes
#include <vector>
#include <iostream>
#include <string>

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
    millimeter_t m_height;    // height of the robot
    millimeter_t m_width;     // width of the robot
    degree_t m_angle;         // angle of the robot
    millimeter_t m_x;         // position 'x' of the robot on the screen
    millimeter_t m_y;         // position 'y' of the robot on the screen

    float mouseClickX;
    float mouseClickY;

    bool quit;
    SDL_Event event;
    SDL_Window *window;
    SDL_Renderer *renderer;
    SDL_Surface *image;
    SDL_Texture *texture;
    SDL_Rect dstrect;
    SDL_Rect rect_goal;

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
    void drawRectangleAtCoordinates(SDL_Rect &rectangle, const float x, const float y) {

        rectangle.x = x;
        rectangle.y = y;
        rectangle.w = 5;
        rectangle.h = 5;

        SDL_SetRenderDrawColor( renderer, 0, 0, 255, 255 );
        SDL_RenderFillRect( renderer, &rectangle );
    }

public:
    Simulator()
      : quit(false)
    {
        SDL_Init(SDL_INIT_VIDEO);

        window = SDL_CreateWindow("Wheeled-robot simulator",
            SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WindowWidth, WindowHeight, 0);

        renderer = SDL_CreateRenderer(window, -1, 0);

        const std::string imagePath = std::string(std::getenv("BOB_ROBOTICS_PATH")) + "/robots/car.bmp";
        image = SDL_LoadBMP(imagePath.c_str());
        BOB_ASSERT(image != nullptr); // Check file exists

        texture = SDL_CreateTextureFromSurface(renderer, image);

        // Select the color for drawing.
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        // Clear the entire screen to our selected color.
        SDL_RenderClear(renderer);

        // initial position and size of the robot car
        dstrect = { WindowWidth / 2, WindowHeight / 2, 10, 13 };

        m_x = dstrect.x * MMPerPixel;
        m_y = dstrect.y * MMPerPixel;
        m_angle = 10_deg;

        // first goal is set to the middle of the window
        mouseClickX = WindowWidth/2;
        mouseClickY = WindowHeight/2;
    }

    ~Simulator()
    {
        // freeing up resources
        SDL_DestroyTexture(texture);
        SDL_FreeSurface(image);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }

    //! sets the current pose of the robot
    void setPose(const millimeter_t x, const millimeter_t y, const degree_t theta)
    {
        m_x = x;
        m_y = y;
        m_angle = theta;

        // Update agent's position in pixels
        dstrect.x = m_x / MMPerPixel;
        dstrect.y = m_y / MMPerPixel;
    }

    //! sets the robot's size in millimeter
    void setRobotSize(const millimeter_t height,
                      const millimeter_t width
                      )
    {
        m_height = height;
        m_width = width;
        dstrect.h = height / MMPerPixel;
        dstrect.w = width / MMPerPixel;
    }

    //! returns true if we did quit the simulator's gui
    bool didQuit() const
    {
        return quit;
    }

    //! suimulates a step of the simulation with the provided velocities
    bool simulationStep(meters_per_second_t v,
                        degrees_per_second_t w,
                        second_t delta_time
                        )
    {
        bool ret = false;

        // Clear the entire screen to our selected color.
        SDL_RenderClear(renderer);
        SDL_PollEvent(&event);

        // getting events
        switch (event.type) {
        case SDL_QUIT:
            quit = true;
            break;

        case SDL_KEYDOWN:
            switch (event.key.keysym.sym) {
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
            if (event.button.button == SDL_BUTTON_LEFT) {
                int x, y;
                SDL_GetMouseState(&x, &y);
                mouseClickX = x;
                mouseClickY = y;
            }
            break;

        default:
            break;
        }

        updatePose(v, w, delta_time);

        // draw a rectangle at the goal position
        drawRectangleAtCoordinates(rect_goal , mouseClickX, mouseClickY);

        // Select the color for drawing.
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        // render texture with rotation
        SDL_RenderCopyEx( renderer, texture, NULL, &dstrect, m_angle.value(), NULL, SDL_FLIP_NONE );

        SDL_RenderPresent(renderer);

        return ret;
    }

    //! returns the current position of the robot
    std::vector<float> getCurrentPosition()
    {
        std::vector<float> position;
        position.push_back(dstrect.x);
        position.push_back(dstrect.y);
        position.push_back(m_angle.value());
        return position;
    }

    //! gets the position of the latest mouse click relative to the window
    std::vector<float> getMouseClickLocation()
    {
        std::vector<float> mouseClickCoordinates;
        mouseClickCoordinates.push_back(mouseClickX);
        mouseClickCoordinates.push_back(mouseClickY);
        return mouseClickCoordinates;
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
