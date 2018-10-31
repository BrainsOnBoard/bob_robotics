// Simulator
#pragma once;
#include <SDL2/SDL.h>
#include "../third_party/units.h"
#include <vector>
#include <iostream>

#define PI 3.14159
#define WINDOW_WIDTH 800.0
#define WINDOW_HEIGHT 600.0


class Simulator {

private:
	float m_v; 												// velocity v (translational velocity)
	float m_w; 												// velocity w (rotational velocity)
	float m_height; 										// height of the robot 
	float m_width; 											// width of the robot 
	float m_angle; 											// angle of the robot
	float m_x; 												// position 'x' of the robot on the screen
	float m_y; 												// position 'y' of the robot on the screen
	float m_dt; 											// delta time 

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


	void updatePose(const float v, const float w) 
	{
		// set current velocities 
		m_v = v;
		m_w = w;

		// calculating next position, given velocity commands - v and w
		m_x     = m_x + (-m_v/m_w) * sin(m_angle*PI/180.0) + ( m_v/m_w) * sin( (m_angle + m_w * m_dt)*PI/180.0 );
		m_y     = m_y + ( m_v/m_w) * cos(m_angle*PI/180.0) - ( m_v/m_w) * cos( (m_angle + m_w * m_dt)*PI/180.0 );
		m_angle += m_w * m_dt;
	}

public:
	Simulator() 
	{
		SDL_Init(SDL_INIT_VIDEO);
 
		window = SDL_CreateWindow("Norbot and friends - simulator",
			SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_WIDTH, WINDOW_HEIGHT, 0);
	
		renderer = SDL_CreateRenderer(window, -1, 0);
		image = SDL_LoadBMP("car.bmp");
		texture = SDL_CreateTextureFromSurface(renderer, image);

		// Select the color for drawing. 
		SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

		// Clear the entire screen to our selected color.
		SDL_RenderClear(renderer);

		// initial position and size of the robot car
		dstrect = { 5, 5, 65, 50 };	  // currently 1cm translates to 5cm

		m_x = dstrect.x;
		m_y = dstrect.y;
		m_angle = 0;
		m_dt = 1;    //change it < (need to time the simulation step)
		m_v = 0;
		m_w = 0;

		// first goal is set to the middle of the window 
		mouseClickX = WINDOW_WIDTH/2;
		mouseClickY = WINDOW_HEIGHT/2;
	}

	~Simulator() 
	{
		
	}

	void setRobotSize(const float height, const float width) 
	{
		m_height = height;
		m_width = width;
		dstrect.h = height;
		dstrect.w = width;
	}

	bool didQuit() {
		return quit;
	}

	//! suimulates a step of the simulation with the provided velocities
	void simulationStep(float v, float w) 
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
						w = -0.5;
						break;
					case SDLK_RIGHT:  
						w = 0.5;
						break;
					case SDLK_UP:     
						v = 10.0;
						w = 0.0001;
						break;
					case SDLK_DOWN:  
						v = -10.0;
						w = 0.0001;
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
				if (v == 0 && w == 0) {
					v = 0.0001;
					w = 0.0001;
				}
				break;
		}

		updatePose(v, w);

		// moving the robot
		dstrect.x = m_x;
		dstrect.y = m_y;

		// draw a rectangle at the goal position
		drawRectangleAtCoordinates(rect_goal , mouseClickX, mouseClickY);

		// Select the color for drawing. 
		SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
		
		// render texture with rotation
		SDL_RenderCopyEx( renderer, texture, NULL, &dstrect, m_angle, NULL, SDL_FLIP_NONE );
		
		SDL_RenderPresent(renderer);
	
	}

	//! draws the rectangle at the desired coordinates
	void drawRectangleAtCoordinates(SDL_Rect &rectangle, const float x, const float y) {
		
		rectangle.x = x;
		rectangle.y = y;
		rectangle.w = 10;
		rectangle.h = 10;

		// Set render color to blue ( rect will be rendered in this color )
		SDL_SetRenderDrawColor( renderer, 0, 0, 255, 255 );

		// Render rect
		SDL_RenderFillRect( renderer, &rectangle );
		
	}

	//! returns the current position of the robot
	std::vector<float> getCurrentPosition() 
	{ 
		std::vector<float> position;
		position.push_back(dstrect.x);
		position.push_back(dstrect.y);
		position.push_back(m_angle);
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


};