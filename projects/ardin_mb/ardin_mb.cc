// Ardin MB includes
#include "mb_memory_ardin.h"
#include "sim_params.h"
#include "state_handler.h"
#include "visual_navigation_ui.h"

// Antworld includes
#include "antworld/snapshot_processor_ardin.h"

// BoB Robotics includes
#include "common/path.h"
#include "common/logging.h"
#include "navigation/infomax.h"
#include "navigation/perfect_memory.h"

// OpenGL includes
#include <GL/glew.h>

// SFML includes
#include <SFML/Graphics.hpp>

// CLI11 includes
#include "third_party/CLI11.hpp"

// IMGUI
#include "imgui.h"
//#include "examples/imgui_impl_sfml.h"
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

//----------------------------------------------------------------------------
// Anonymous namespace
//----------------------------------------------------------------------------
namespace
{
}   // anonymous namespace



int main(int argc, char *argv[])
{
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


    // Set clear colour to match matlab and enable depth test
    glClearColor(0.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glLineWidth(4.0);
    glPointSize(4.0);

    const char *bobRoboticsPath = std::getenv("BOB_ROBOTICS_PATH");
    assert(bobRoboticsPath != nullptr);

    // Default parameters"
    std::string worldFilename = std::string(bobRoboticsPath) + "/resources/antworld/world5000_gray.bin";
    std::string routeFilename = "";
    std::string logFilename = "";
    float jitterSD = 0.0f;
    bool quitAfterTrain = false;
    bool autoTest = false;

    CLI::App app{"Mushroom body navigation model"};
    app.add_option("--jitter", jitterSD, "Amount of jitter (cm) to apply when recapitulating routes", true);
    app.add_option("--world", worldFilename, "File to load world from", true);
    app.add_option("--log", logFilename, "File to log to", true);
    app.add_flag("--quit-after-train", quitAfterTrain, "Whether to quit once model is trained");
    app.add_flag("--auto-test", autoTest, "Whether to test the model once it's trained");
    app.add_option("route", routeFilename, "Filename of route");

    /*cv::Size unwrapRes(std::atoi(argv[2]), std::atoi(argv[3]));
    cv::Size cellSize(std::atoi(argv[4]), std::atoi(argv[5]));
    int numOrientation = std::atoi(argv[6]);*/

    //std::cout << "Unwrap res: (" << unwrapRes.width << ", " << unwrapRes.height << "), cell size:(" << cellSize.width << "," << cellSize.height << "), num orientations:" << numOrientation << ", jitter sd:" << jitterSD << std::endl;*/
    // Create memory
    //Navigation::PerfectMemory<Navigation::PerfectMemoryStore::HOG<>> memory(cv::Size(MBParams::inputWidth, MBParams::inputHeight),
    //                                                                        cv::Size(5, 5), 2);
    //Navigation::PerfectMemory<Navigation::PerfectMemoryStore::HOG<>> memory(cv::Size(MBParams::inputWidth, MBParams::inputHeight),
    //                                                                        cv::Size(6, 6), cv::Size(4, 4), 4);
    //Navigation::PerfectMemory<Navigation::PerfectMemoryStore::HOG<>> memory(unwrapRes, cellSize, numOrientation);
    //Navigation::InfoMax<float> memory(cv::Size(MBParams::inputWidth, MBParams::inputHeight), 0.01f);
    //VisualNavigationUI ui;
    //MBMemory memory;


    /*Navigation::PerfectMemory<> memory(cv::Size(36, 10));
    VisualNavigationUI ui;
    //AntWorld::SnapshotProcessorArdin snapshotProcessor(8, 74, 19,
    //                                                   memory.getUnwrapResolution().width, memory.getUnwrapResolution().height);
    AntWorld::SnapshotProcessorSegmentSky snapshotProcessor(memory.getUnwrapResolution().width,
                                                            memory.getUnwrapResolution().height);*/
    // Mushroom body
    MBMemoryArdin memory;
    memory.addCLIArguments(app);
    MBArdinUI ui(memory);

    AntWorld::SnapshotProcessorArdin snapshotProcessor(8, 74, 19,
                                                       memory.getUnwrapResolution().width, memory.getUnwrapResolution().height);

    // Parse command line arguments
    CLI11_PARSE(app, argc, argv);

    // Create state machine and set it as window user pointer
    StateHandler stateHandler(worldFilename, routeFilename, jitterSD, quitAfterTrain, autoTest,
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
/*
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
            break;Sfml
        }*/
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
