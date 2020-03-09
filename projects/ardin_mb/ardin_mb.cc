// BoB Robotics includes
#include "common/logging.h"
#include "navigation/infomax.h"
#include "navigation/perfect_memory.h"

// Ardin MB includes
#include "mb_memory.h"
#include "mb_params.h"
#include "sim_params.h"
#include "state_handler.h"

// CLI11 includes
#include "third_party/CLI11.hpp"

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
    // Default parameters"
    std::string worldFilename = "";
    std::string routeFilename = "";
    std::string logFilename = "";
    std::string perceptionStorage = "";
    float heightMetres = 0.01f;
    std::vector<float> minBound;
    std::vector<float> maxBound;
    std::vector<float> clearColour{0.0f, 1.0f, 1.0f, 1.0f};

    CLI::App app{"Mushroom body navigation model"};
    app.add_option("--world", worldFilename, "File to load world from", true);
    app.add_option("--height", heightMetres, "Height in metres to navigate at", true);
    app.add_option("--min-bound", minBound, "Override default world min bound with this one", true)->expected(3);
    app.add_option("--max-bound", maxBound, "Override default world max bound with this one", true)->expected(3);
    app.add_option("--clear-colour", clearColour, "Set background colour used for rendering", true)->expected(4);
    app.add_option("route", routeFilename, "Filename of route");
    app.add_option("--perception-storage", perceptionStorage, "Identifyer for Store class");

    // Parse command line arguments
    CLI11_PARSE(app, argc, argv);

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

    // Tweak OpenGL settings
    glClearColor(clearColour[0], clearColour[1], clearColour[2], clearColour[3]);
    glEnable(GL_DEPTH_TEST);
    glLineWidth(4.0);
    glPointSize(4.0);

// Create memory
#ifdef NO_GENN
    Navigation::PerfectMemory<Navigation::PerfectMemoryStore::WVC> memory(cv::Size(MBParams::inputWidth, MBParams::inputHeight));
    //Navigation::PerfectMemory<> memory(cv::Size(MBParams::inputWidth, MBParams::inputHeight));    
    
    //Navigation::InfoMax<float> memory(cv::Size(MBParams::inputWidth, MBParams::inputHeight), 0.01f);
#else
    MBMemory memory;
#endif



    // Create state machine
    StateHandler stateHandler(worldFilename, routeFilename, units::length::meter_t{heightMetres},
                              minBound, maxBound, memory);

    // Loop until window should close
    sf::Event event;
    for (unsigned int frame = 0; window.isOpen(); frame++) {
        // Process events
        while (window.pollEvent(event)) {
            // Close window: exit
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            // Key events
            else if(event.type == sf::Event::KeyPressed || event.type == sf::Event::KeyReleased) {
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

                    case sf::Keyboard::Key::Return:
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

        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Update
        if(!stateHandler.update()) {
            break;
        }

        // Swap front and back buffers
        window.display();
    }

    return 0;
}
