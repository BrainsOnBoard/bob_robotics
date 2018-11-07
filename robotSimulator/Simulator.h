// Simulator
#pragma once;
#include <SDL2/SDL.h>
#include "../third_party/units.h"
#include <vector>
#include <iostream>

#define PI 3.14159
#define WINDOW_WIDTH 800.0
#define WINDOW_HEIGHT 600.0
#define SCALE_FACTOR 0.25

using namespace units::literals;


class Simulator {

private:

    units::velocity::meters_per_second_t m_v;               // velocity v (translational velocity)
    units::angular_velocity::degrees_per_second_t m_w;      // velocity w (rotational velocity)
    units::length::millimeter_t m_height;                   // height of the robot 
    units::length::millimeter_t m_width;                    // width of the robot 
    units::angle::degree_t m_angle;                         // angle of the robot
    units::length::millimeter_t m_x;                        // position 'x' of the robot on the screen
    units::length::millimeter_t m_y;                        // position 'y' of the robot on the screen
    units::time::millisecond_t m_dt;                        // delta time 

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

    void updatePose(const units::velocity::meters_per_second_t v, 
                    const units::angular_velocity::degrees_per_second_t w, 
                    const units::time::millisecond_t dt) 
    {
        // set current velocities 
        m_v = v;
        m_w = w;

        // calculating next position, given velocity commands - v and w
        auto x_part = (-v/w) * units::math::sin(m_angle) + ( v/w) * units::math::sin( (m_angle + w * dt) );
        auto y_part = ( v/w) * units::math::cos(m_angle) - ( v/w) * units::math::cos( (m_angle + w * dt) );
        m_x += units::length::meter_t(x_part.value());
        m_y += units::length::meter_t(y_part.value());
        m_angle += w * dt;

    }

public:
    Simulator() 
    {
        SDL_Init(SDL_INIT_VIDEO);
 
        window = SDL_CreateWindow("Wheeled-robot simulator",
            SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    
        renderer = SDL_CreateRenderer(window, -1, 0);
        image = SDL_LoadBMP("car.bmp");
        texture = SDL_CreateTextureFromSurface(renderer, image);

        // Select the color for drawing. 
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        // Clear the entire screen to our selected color.
        SDL_RenderClear(renderer);

        // initial position and size of the robot car
        dstrect = { 5, 5, 10, 13 };	  

        m_x = units::length::millimeter_t((float)dstrect.x *SCALE_FACTOR);
        m_y = units::length::millimeter_t((float)dstrect.y *SCALE_FACTOR);
        m_angle = 10_deg;
        
        // first goal is set to the middle of the window 
        mouseClickX = WINDOW_WIDTH/2;
        mouseClickY = WINDOW_HEIGHT/2;
    }

    ~Simulator() {}

    void setRobotSize(const units::length::millimeter_t height, 
                      const units::length::millimeter_t width
                      ) 
    {
        m_height = height;
        m_width = width;
        dstrect.h = height.value() * SCALE_FACTOR;
        dstrect.w = width.value() * SCALE_FACTOR;
    }

    bool didQuit() {
        return quit;
    }

    //! suimulates a step of the simulation with the provided velocities
    void simulationStep(units::velocity::meters_per_second_t v, 
                        units::angular_velocity::degrees_per_second_t w,
                        units::time::millisecond_t delta_time
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
                        w = units::angular_velocity::degrees_per_second_t(-35.0);
                        break;
                    case SDLK_RIGHT:  
                        w = units::angular_velocity::degrees_per_second_t( 35.0);
                        break;
                    case SDLK_UP:     
                        v = units::velocity::meters_per_second_t(5.0);
                        w = units::angular_velocity::degrees_per_second_t(1e-10);
                        break;
                    case SDLK_DOWN:  
                        v = units::velocity::meters_per_second_t(5.0);
                        w = units::angular_velocity::degrees_per_second_t(1e-10);
                        break;
                }
            
            case SDL_MOUSEBUTTONDOWN:
                // If the left button was pressed. 
                if (event.button.button == SDL_BUTTON_LEFT) {
                    int x, y;
                    SDL_GetMouseState(&x, &y);
                    mouseClickX = x;
                    mouseClickY = y;			
                }
                       
            default: 
                // adding a small value to avoid dividing by 0
                if (v.value() == 0 && w.value() == 0) {
                    v = units::velocity::meters_per_second_t(1e-10);
                    w = units::angular_velocity::degrees_per_second_t(1e-10);
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

    //! draws the rectangle at the desired coordinates
    void drawRectangleAtCoordinates(SDL_Rect &rectangle, const float x, const float y) {
        
        rectangle.x = x;
        rectangle.y = y;
        rectangle.w = 5;
        rectangle.h = 5;

        SDL_SetRenderDrawColor( renderer, 0, 0, 255, 255 );

    
        SDL_RenderFillRect( renderer, &rectangle );
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

    //! gets the position of the mouse relative to the window
    std::vector<float> getMouseClickLocation() 
    {
        std::vector<float> mouseClickCoordinates;
        mouseClickCoordinates.push_back(mouseClickX);
        mouseClickCoordinates.push_back(mouseClickY);
        return mouseClickCoordinates;
    }

    static void changePixelToMM(const float x_pos,
                         const float y_pos,
                         units::length::millimeter_t &x_mm,
                         units::length::millimeter_t &y_mm) 
    {
        x_mm = units::length::millimeter_t(x_pos / SCALE_FACTOR);
        y_mm = units::length::millimeter_t(y_pos / SCALE_FACTOR);
    }

};