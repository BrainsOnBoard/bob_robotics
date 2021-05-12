// Ardin MB includes
#ifndef NO_GENN
    #include "mb_memory_ardin.h"
#endif
#include "sim_params.h"
#include "state_handler.h"
#include "visual_navigation_bob.h"
#include "visual_navigation_perfect_memory_window.h"
#include "visual_navigation_ui.h"

// Antworld includes
#include "antworld/snapshot_processor_ardin.h"

// BoB Robotics includes
#include "navigation/infomax.h"
#include "navigation/perfect_memory.h"
#include "navigation/perfect_memory_window.h"

// PLOG includes
#include "plog/Log.h"

// OpenGL includes
#include <GL/glew.h>

// SFML includes
#include <SFML/Graphics.hpp>

// CLI11 includes
#include "third_party/CLI11.hpp"

// IMGUI
#include "imgui.h"
#include "imgui_impl_sfml.h"
#include "examples/imgui_impl_opengl2.h"

// Standard C++ includes
#include <bitset>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <tuple>
#include <vector>

// Standard C includes
#include <cmath>

using namespace BoBRobotics;

int bobMain(int argc, char **argv)
{
    // Default parameters"
    std::string worldFilename = "";
    std::string routeFilename = "";
    std::string logFilename = "";
    float jitterSD = 0.0f;
    bool quitAfterTrain = false;
    bool autoTest = false;
    float heightMetres = 0.01f;
    std::vector<float> minBound;
    std::vector<float> maxBound;
    std::vector<float> clearColour{0.0f, 1.0f, 1.0f, 1.0f};

    CLI::App app{"Mushroom body navigation model"};
    app.add_option("--jitter", jitterSD, "Amount of jitter (cm) to apply when recapitulating routes", true);
    app.add_option("--world", worldFilename, "File to load world from", true);
    app.add_option("--log", logFilename, "File to log to", true);
    app.add_flag("--quit-after-train", quitAfterTrain, "Whether to quit once model is trained");
    app.add_flag("--auto-test", autoTest, "Whether to test the model once it's trained");
    app.add_option("--height", heightMetres, "Height in metres to navigate at", true);
    app.add_option("--min-bound", minBound, "Override default world min bound with this one", true)->expected(3);
    app.add_option("--max-bound", maxBound, "Override default world max bound with this one", true)->expected(3);
    app.add_option("--clear-colour", clearColour, "Set background colour used for rendering", true)->expected(4);
    app.add_option("route", routeFilename, "Filename of route");

    // Parse command line arguments
    CLI11_PARSE(app, argc, argv);

    // Create SFML window
    sf::Window window(sf::VideoMode(1440, 1024),
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

    // Turn off padding so weird size textures render correctly in OGL
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // Setup ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer bindings
    ImGui_ImplSfml_Init(&window);
    ImGui_ImplOpenGL2_Init();

#ifdef NO_GENN
    //VisualNavigationBoB<Navigation::PerfectMemory<>> memory(cv::Size(36, 10));
    VisualNavigationPerfectMemoryWindow<> memory(std::make_unique<Navigation::PerfectMemoryWindow::Fixed>(10),
                                                 cv::Size(36, 10));
    VisualNavigationUI ui;
#else
    // Mushroom body
    MBMemoryArdin memory;
    memory.addCLIArguments(app);
    MBArdinUI ui(memory);
#endif
    AntWorld::SnapshotProcessorArdin snapshotProcessor(8, 74, 19,
                                                       memory.getUnwrapResolution().width, memory.getUnwrapResolution().height);

    // Parse command line arguments
    CLI11_PARSE(app, argc, argv);

    // Tweak OpenGL settings
    glClearColor(clearColour[0], clearColour[1], clearColour[2], clearColour[3]);
    glEnable(GL_DEPTH_TEST);
    glLineWidth(4.0);
    glPointSize(4.0);

    // Create state machine and set it as window user pointer
    StateHandler stateHandler(worldFilename, routeFilename, jitterSD, quitAfterTrain, autoTest,
                              units::length::meter_t{heightMetres}, minBound, maxBound,
                              snapshotProcessor, memory, ui);

     // Loop until window should close
    for(unsigned int frame = 0; window.isOpen(); frame++) {
        // Process events
        sf::Event event;
        while (window.pollEvent(event)) {
            // Close window: exit
            if (event.type == sf::Event::Closed) {
                window.close();
            }
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
                    stateHandler.setKeyState(StateHandler::KeyForward, pressed);
                    break;

                case sf::Keyboard::Key::Down:
                    stateHandler.setKeyState(StateHandler::KeyBackward, pressed);
                    break;

                case sf::Keyboard::Key::R:
                    stateHandler.setKeyState(StateHandler::KeyReset, pressed);
                    break;
                default:
                    break;
                }
            }
            else {
                ImGui_ImplSfml_ProcessEvent(event);
            }
        }


        // Start ImGui frame
        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplSfml_NewFrame();
        ImGui::NewFrame();

        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Update
        if(!stateHandler.update()) {
            break;
        }

        // Generate ImGUI geometry
        ImGui::Render();

        // Render UI
        const auto displaySize = window.getSize();
        glViewport(0, 0, displaySize.x, displaySize.y);
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        window.display();
    }

    // Save logs
    // **YUCK** this probably shouldn't be part of UI
    ui.saveLogs(logFilename);

     // Cleanup UI
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplSfml_Shutdown();
    ImGui::DestroyContext();

    return EXIT_SUCCESS;
}
