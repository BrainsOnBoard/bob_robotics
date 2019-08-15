// This is the main file, so we do want header definitions for this object
#undef NO_HEADER_DEFINITIONS

// BoB Robotics includes
#include "common/logging.h"
#include "navigation/infomax.h"
#include "navigation/perfect_memory.h"

// Ardin MB includes
#include "mb_memory.h"
#include "mb_params.h"
#include "sim_params.h"
#include "state_handler.h"

// OpenGL includes
#include <GL/glew.h>

// SFML includes
#include <SFML/Graphics.hpp>

// Standard C++ includes
#include <bitset>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <tuple>
#include <vector>

// Standard C includes
#include <cmath>

using namespace BoBRobotics;

int main(int argc, char *argv[])
{
    // Create SFML window
    sf::Window window(sf::VideoMode(SimParams::displayRenderWidth, SimParams::displayRenderHeight + SimParams::displayRenderWidth + 10),
                      "Ant world",
                      sf::Style::Titlebar | sf::Style::Close);

    // Enable VSync
    window.setVerticalSyncEnabled(true);
    window.setActive(true);

    // Initialize GLEW
    if(glewInit() != GLEW_OK) {
        LOGE << "Failed to initialize GLEW";
        return EXIT_FAILURE;
    }

    // Set clear colour to match matlab and enable depth test
    glClearColor(0.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glLineWidth(4.0);
    glPointSize(4.0);

    const char *bobRoboticsPath = std::getenv("BOB_ROBOTICS_PATH");
    assert(bobRoboticsPath != nullptr);

    // Create memory
    //Navigation::PerfectMemory<> memory(cv::Size(MBParams::inputWidth, MBParams::inputHeight));
    //Navigation::InfoMax<float> memory(cv::Size(MBParams::inputWidth, MBParams::inputHeight), 0.01f);
    MBMemory memory;

    // Create state machine and set it as window user pointer
    const std::string worldFilename = std::string(bobRoboticsPath) + "/resources/antworld/world5000_gray.bin";
    const std::string routeFilename = (argc > 1) ? argv[1] : "";
    StateHandler stateHandler(worldFilename, routeFilename, memory);

    // Loop until window should close
    sf::Event event;
    for (unsigned int frame = 0; window.isOpen(); frame++) {
        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Update
        if(!stateHandler.update()) {
            break;
        }

        // Swap front and back buffers
        window.display();

        // Poll for and process events
        if (window.pollEvent(event) && (event.type == sf::Event::KeyPressed || event.type == sf::Event::KeyReleased)) {
            const bool pressed = event.type == sf::Event::KeyPressed;

            // Apply new key state to bits of key bits
            switch(event.key.code) {
            case sf::Keyboard::Key::Left:
                stateHandler.setKeyState(StateHandler::KeyLeft, pressed);
                break;

            case sf::Keyboard::Key::Right:
                stateHandler.setKeyState(StateHandler::KeyRight, pressed);
                break;

            case sf::Keyboard::Key::Up:
                stateHandler.setKeyState(StateHandler::KeyUp, pressed);
                break;

            case sf::Keyboard::Key::Down:
                stateHandler.setKeyState(StateHandler::KeyDown, pressed);
                break;

            case sf::Keyboard::Key::R:
                stateHandler.setKeyState(StateHandler::KeyReset, pressed);
                break;

            case sf::Keyboard::Key::Space:
                stateHandler.setKeyState(StateHandler::KeyTrainSnapshot, pressed);
                break;

            case sf::Keyboard::Key::Enter:
                stateHandler.setKeyState(StateHandler::KeyTestSnapshot, pressed);
                break;

            case sf::Keyboard::Key::S:
                stateHandler.setKeyState(StateHandler::KeySaveSnapshot, pressed);
                break;

            case sf::Keyboard::Key::W:
                stateHandler.setKeyState(StateHandler::KeyRandomWalk, pressed);
                break;

            case sf::Keyboard::Key::V:
                stateHandler.setKeyState(StateHandler::KeyBuildVectorField, pressed);
                break;
            default:
                break;
            }
        }
    }

    return 0;
}
