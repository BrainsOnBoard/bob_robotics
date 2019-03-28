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
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Robots {
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
    CarDisplay(const millimeter_t screenHeight = 3.2_m, const millimeter_t carWidth = 16.4_cm)
      : m_MMPerPixel(screenHeight / WindowHeight)
    {
        SDL_Init(SDL_INIT_VIDEO);

        m_Window = SDL_CreateWindow("Wheeled-robot simulator",
                                    SDL_WINDOWPOS_UNDEFINED,
                                    SDL_WINDOWPOS_UNDEFINED,
                                    WindowWidth,
                                    WindowHeight,
                                    0);

        m_Renderer = SDL_CreateRenderer(m_Window, -1, 0);

        const std::string imagePath = std::string(std::getenv("BOB_ROBOTICS_PATH")) + "/robots/car.bmp";
        SDL_Surface *image = SDL_LoadBMP(imagePath.c_str());
        BOB_ASSERT(image != nullptr); // Check file exists

        // Turn bitmap into texture
        m_Texture = SDL_CreateTextureFromSurface(m_Renderer, image);
        SDL_FreeSurface(image); // We're done with bitmap

        // Select the color for drawing.
        SDL_SetRenderDrawColor(m_Renderer, 255, 255, 255, 255);

        // Clear the entire screen to our selected color.
        SDL_RenderClear(m_Renderer);

        // initial position and size of the robot car
        const double widthPx = carWidth / m_MMPerPixel;
        m_RobotRectangle.h = static_cast<int>(widthPx);
        m_RobotRectangle.w = static_cast<int>((444.0 / 208.0) * widthPx);

        // first goal is set to the middle of the window
        m_MouseClickPosition[0] = WindowWidth / 2;
        m_MouseClickPosition[1] = WindowHeight / 2;

        setRobotPosition(0_m, 0_m);
    }

    ~CarDisplay()
    {
        // freeing up resources
        SDL_DestroyTexture(m_Texture);
        SDL_DestroyRenderer(m_Renderer);
        SDL_DestroyWindow(m_Window);
        SDL_Quit();
    }

    //! Returns true if GUI is still running
    bool isOpen() const
    {
        return m_IsOpen;
    }

    auto runGUI(const Pose2<millimeter_t, degree_t> &agentPose)
    {
        const auto key = pollEvents();

        // Update agent's position in pixels
        setRobotPosition(agentPose.x(), agentPose.y());

        draw(agentPose.yaw());

        return key;
    }

    Vector2<millimeter_t> getMouseClickPosition() const
    {
        Vector2<millimeter_t> out;
        pixelToMM(m_MouseClickPosition[0], m_MouseClickPosition[1], out[0], out[1]);
        return out;
    }

    std::vector<float> getMouseClickPixelPosition() const
    {
        std::vector<float> out;
        out.push_back(m_MouseClickPosition[0]);
        out.push_back(m_MouseClickPosition[1]);
        return out;
    }

    //! draws the rectangle at the desired coordinates
    void drawRectangleAtCoordinates(const SDL_Rect &rectangle)
    {
        SDL_SetRenderDrawColor(m_Renderer, 0, 0, 255, 255);
        SDL_RenderFillRect(m_Renderer, &rectangle);
    }

    SDL_Renderer* getRenderer() {
        return m_Renderer;
    }

    void clearScreen() {
        // Clear the entire screen to our selected color.
        SDL_RenderClear(m_Renderer);
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

    std::pair<SDL_Keycode, bool> pollEvents()
    {
        // getting events
        static SDL_Event event;
        SDL_PollEvent(&event);
        switch (event.type) {
        case SDL_QUIT:
            m_IsOpen = false;
            break;
        case SDL_KEYDOWN:
            return std::make_pair(event.key.keysym.sym, true);
        case SDL_KEYUP:
            return std::make_pair(event.key.keysym.sym, false);
        case SDL_MOUSEBUTTONDOWN:
            // If the left button was pressed.
            if (event.button.button == SDL_BUTTON_LEFT) {
                SDL_GetMouseState(&m_MouseClickPosition[0], &m_MouseClickPosition[1]);
            }
            break;
        default:
            break;
        }

        return std::make_pair(0, false);
    }


   

    void draw(const degree_t agentAngle)
    {
        // Clear the entire screen to our selected color.
        //SDL_RenderClear(m_Renderer);

        // draw a rectangle at the goal position
        drawRectangleAtCoordinates({ m_MouseClickPosition[0], m_MouseClickPosition[1], 5, 5 });

        // Select the color for drawing.
        SDL_SetRenderDrawColor(m_Renderer, 255, 255, 255, 255);

        // render texture with rotation
        SDL_RenderCopyEx(m_Renderer, m_Texture, nullptr, &m_RobotRectangle,
                         -agentAngle.value(), nullptr, SDL_FLIP_NONE);

        SDL_RenderPresent(m_Renderer);
    }

    void setRobotPosition(const millimeter_t x, const millimeter_t y)
    {
        mmToPixel(x, y, m_RobotRectangle.x, m_RobotRectangle.y);
        m_RobotRectangle.x -= m_RobotRectangle.w / 2;
        m_RobotRectangle.y -= m_RobotRectangle.h / 2;
    }
}; // CarDisplay

#ifndef NO_HEADER_DEFINITIONS
constexpr int CarDisplay::WindowWidth, CarDisplay::WindowHeight;
#endif
} // Robots
} // BoBRobotics
