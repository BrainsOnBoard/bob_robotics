#pragma once

// Third-party includes
#include "../third_party/units.h"

// SDL
#include <SDL2/SDL.h>

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <vector>
#include <iostream>
#include <string>

#define PI 3.14159
#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define SCALE_FACTOR 0.25


namespace BoBRobotics {
namespace Robots {

using namespace units::literals;
class Simulator {
private:
    using millimeter_t = units::length::millimeter_t;
    using degree_t = units::angle::degree_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;
    using millisecond_t = units::time::millisecond_t;

    // Agent's forward velocity and turning speed
    static constexpr meters_per_second_t Velocity = 5_mps;
    static constexpr degrees_per_second_t TurnSpeed = 50_deg_per_s;

    meters_per_second_t m_v;  // velocity v (translational velocity)
    degrees_per_second_t m_w; // velocity w (rotational velocity)
    millimeter_t m_height;    // height of the robot
    millimeter_t m_width;     // width of the robot
    degree_t m_angle;         // angle of the robot
    millimeter_t m_x;         // position 'x' of the robot on the screen
    millimeter_t m_y;         // position 'y' of the robot on the screen
    millisecond_t m_dt;       // delta time

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
                    const millisecond_t dt)
    {
        // set current velocities
        m_v = v;
        m_w = w;

        // calculating next position, given velocity commands - v and w
        using namespace units::math;
        auto x_part = (-v / w) * sin(m_angle) + (v / w) * sin((m_angle + w * dt));
        auto y_part = ( v/w) * cos(m_angle) - ( v/w) * cos( (m_angle + w * dt) );
        m_x += units::length::meter_t(x_part.value());
        m_y += units::length::meter_t(y_part.value());
        m_angle += w * dt;
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
            SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_WIDTH, WINDOW_HEIGHT, 0);

        renderer = SDL_CreateRenderer(window, -1, 0);
        const std::string imagePath = std::string(std::getenv("BOB_ROBOTICS_PATH")) + "/robots/car.bmp";
        image = SDL_LoadBMP(imagePath.c_str());
        texture = SDL_CreateTextureFromSurface(renderer, image);

        // Select the color for drawing.
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        // Clear the entire screen to our selected color.
        SDL_RenderClear(renderer);

        // initial position and size of the robot car
        dstrect = { WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 10, 13 };

        m_x = millimeter_t((float)dstrect.x *SCALE_FACTOR);
        m_y = millimeter_t((float)dstrect.y *SCALE_FACTOR);
        m_angle = 10_deg;

        // first goal is set to the middle of the window
        mouseClickX = WINDOW_WIDTH/2;
        mouseClickY = WINDOW_HEIGHT/2;
    }

    ~Simulator() {}

    //! sets the robot's size in millimeter
    void setRobotSize(const millimeter_t height,
                      const millimeter_t width
                      )
    {
        m_height = height;
        m_width = width;
        dstrect.h = height.value() * SCALE_FACTOR;
        dstrect.w = width.value() * SCALE_FACTOR;
    }

    //! returns true if we did quit the simulator's gui
    bool didQuit() {
        return quit;
    }

    //! suimulates a step of the simulation with the provided velocities
    void simulationStep(meters_per_second_t v,
                        degrees_per_second_t w,
                        millisecond_t delta_time
                        )
    {
         // Clear the entire screen to our selected color.
        SDL_RenderClear(renderer);
        SDL_PollEvent(&event);

        // getting events
        switch (event.type) {
        case SDL_QUIT:
            quit = true;
            // freeing up resources
            SDL_DestroyTexture(texture);
            SDL_FreeSurface(image);
            SDL_DestroyRenderer(renderer);
            SDL_DestroyWindow(window);
            SDL_Quit();
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
                w = 1e-10_deg_per_s;
                break;
            case SDLK_DOWN:
                v = -Velocity;
                w = 1e-10_deg_per_s;
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
            // adding a small value to avoid dividing by 0
            if (v == 0_mps && w == 0_deg_per_s) {
                v = 1e-10_mps;
                w = 1e-10_deg_per_s;
            }
            break;
        }

        updatePose(v, w, delta_time);

        // moving the robot
        dstrect.x = m_x.value() /SCALE_FACTOR;
        dstrect.y = m_y.value() /SCALE_FACTOR;

        // draw a rectangle at the goal position
        drawRectangleAtCoordinates(rect_goal , mouseClickX, mouseClickY);

        // Select the color for drawing.
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        // render texture with rotation
        SDL_RenderCopyEx( renderer, texture, NULL, &dstrect, m_angle.value(), NULL, SDL_FLIP_NONE );

        SDL_RenderPresent(renderer);
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
        x_mm = millimeter_t(x_pos / SCALE_FACTOR);
        y_mm = millimeter_t(y_pos / SCALE_FACTOR);
    }
}; // Simulator
} // Robots
} // BoBRobotics
