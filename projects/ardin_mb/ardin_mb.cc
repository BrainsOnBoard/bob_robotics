#undef NO_HEADER_DEFINITIONS

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

// OpenGL includes
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// CLI11 includes
#include "third_party/CLI11.hpp"

// IMGUI
#include "imgui.h"
#include "examples/imgui_impl_glfw.h"
#include "examples/imgui_impl_opengl2.h"

// BoB Robotics includes
#include "navigation/infomax.h"
#include "navigation/perfect_memory.h"
#include "navigation/perfect_memory_store_hog.h"

// Ardin MB includes
#include "mb_memory_hog.h"
#include "mb_params.h"
#include "sim_params.h"
#include "state_handler.h"
#include "visual_navigation_ui.h"

using namespace BoBRobotics;

//----------------------------------------------------------------------------
// Anonymous namespace
//----------------------------------------------------------------------------
namespace
{
void keyCallback(GLFWwindow *window, int key, int, int action, int)
{
    // If action isn't a press or a release, do nothing
    if(action != GLFW_PRESS && action != GLFW_RELEASE) {
        return;
    }

    // Determine what state key bit should be set to
    const bool newKeyState = (action == GLFW_PRESS);

    // Extract key bitset from window's user pointer
    StateHandler *stateHandler = reinterpret_cast<StateHandler*>(glfwGetWindowUserPointer(window));

    // Apply new key state to bits of key bits
    switch(key) {
        case GLFW_KEY_LEFT:
            stateHandler->setKeyState(StateHandler::KeyLeft, newKeyState);
            break;

        case GLFW_KEY_RIGHT:
            stateHandler->setKeyState(StateHandler::KeyRight, newKeyState);
            break;

        case GLFW_KEY_UP:
            stateHandler->setKeyState(StateHandler::KeyUp, newKeyState);
            break;

        case GLFW_KEY_DOWN:
            stateHandler->setKeyState(StateHandler::KeyDown, newKeyState);
            break;

        case GLFW_KEY_R:
            stateHandler->setKeyState(StateHandler::KeyReset, newKeyState);
            break;

    }
}
//----------------------------------------------------------------------------
void handleGLFWError(int errorNumber, const char *message)
{
    std::cerr << "GLFW error number:" << errorNumber << ", message:" << message << std::endl;
}
}   // anonymous namespace


int main(int argc, char *argv[])
{
    // Set GLFW error callback
    glfwSetErrorCallback(handleGLFWError);

    // Initialize the library
    if(!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    // Prevent window being resized
    glfwWindowHint(GLFW_RESIZABLE, false);

    // Create a windowed mode window and its OpenGL context
    GLFWwindow *window = glfwCreateWindow(1440, 1024,
                                          "Ant World", nullptr, nullptr);
    if(!window)
    {
        glfwTerminate();
        throw std::runtime_error("Failed to create window");
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    // Initialize GLEW
    if(glewInit() != GLEW_OK) {
        throw std::runtime_error("Failed to initialize GLEW");
    }

    // Enable VSync
    glfwSwapInterval(1);

    // Setup ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();

    // Set clear colour to match matlab and enable depth test
    glClearColor(0.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glLineWidth(4.0);
    glPointSize(4.0);

    const char *bobRoboticsPath = std::getenv("BOB_ROBOTICS_PATH");
    assert(bobRoboticsPath != nullptr);

    // Default parameters"
    std::string worldFilename = std::string(bobRoboticsPath) + "/libantworld/world5000_gray.bin";
    std::string routeFilename = "";
    std::string logFilename = "";
    float jitterSD = 0.0f;
    bool quitAfterTrain = false;

    CLI::App app{"Mushroom body navigation model"};
    app.add_option("--jitter", jitterSD, "Amount of jitter (cm) to apply when recapitulating routes", true);
    app.add_option("--world", worldFilename, "File to load world from", true);
    app.add_option("--log", logFilename, "File to log to", true);
    app.add_flag("--quit-after-train", quitAfterTrain, "Whether to quit once model is trained");
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
    //Navigation::PerfectMemory<> memory(cv::Size(MBParams::inputWidth, MBParams::inputHeight));
    //Navigation::InfoMax<float> memory(cv::Size(MBParams::inputWidth, MBParams::inputHeight), 0.01f);
    //MBMemory memory;
    MBMemoryHOG memory;
    memory.addCLIArguments(app);

    // Create UI
    MBHogUI ui(memory);

    // Parse command line arguments
    CLI11_PARSE(app, argc, argv);

    // Create state machine and set it as window user pointer
    StateHandler stateHandler(worldFilename, routeFilename, jitterSD, quitAfterTrain, memory, ui);
    glfwSetWindowUserPointer(window, &stateHandler);

    // Set key callback
    glfwSetKeyCallback(window, keyCallback);

     // Loop until window should close
    for(unsigned int frame = 0; !glfwWindowShouldClose(window); frame++) {
        // Poll for and process events
        glfwPollEvents();

        // Start ImGui frame
        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
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
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        // Swap front and back buffers
        glfwSwapBuffers(window);
    }

    // Save logs
    // **YUCK** this probably shouldn't be part of UI
    ui.saveLogs(logFilename);

     // Cleanup UI
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
    return 0;
}
