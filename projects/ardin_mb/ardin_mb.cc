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

// BoB Robotics includes
#include "navigation/infomax.h"
#include "navigation/perfect_memory.h"
#include "navigation/perfect_memory_store_hog.h"

// Ardin MB includes
#include "mb_memory_hog.h"
#include "mb_params.h"
#include "sim_params.h"
#include "state_handler.h"

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

        case GLFW_KEY_SPACE:
            stateHandler->setKeyState(StateHandler::KeyTrainSnapshot, newKeyState);
            break;

        case GLFW_KEY_ENTER:
            stateHandler->setKeyState(StateHandler::KeyTestSnapshot, newKeyState);
            break;

        case GLFW_KEY_S:
            stateHandler->setKeyState(StateHandler::KeySaveSnapshot, newKeyState);
            break;

        case GLFW_KEY_W:
            stateHandler->setKeyState(StateHandler::KeyRandomWalk, newKeyState);
            break;

        case GLFW_KEY_V:
            stateHandler->setKeyState(StateHandler::KeyBuildVectorField, newKeyState);
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
    GLFWwindow *window = glfwCreateWindow(SimParams::displayRenderWidth, SimParams::displayRenderHeight + SimParams::displayRenderWidth + 10,
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

    // Set clear colour to match matlab and enable depth test
    glClearColor(0.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glLineWidth(4.0);
    glPointSize(4.0);

    const char *bobRoboticsPath = std::getenv("BOB_ROBOTICS_PATH");
    assert(bobRoboticsPath != nullptr);

    /*cv::Size unwrapRes(std::atoi(argv[2]), std::atoi(argv[3]));
    cv::Size cellSize(std::atoi(argv[4]), std::atoi(argv[5]));
    int numOrientation = std::atoi(argv[6]);*/
    float jitterSD = std::atof(argv[2]);
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

    // Create state machine and set it as window user pointer
    const std::string worldFilename = std::string(bobRoboticsPath) + "/libantworld/world5000_gray.bin";
    const std::string routeFilename = (argc > 1) ? argv[1] : "";
    StateHandler stateHandler(worldFilename, routeFilename, jitterSD, memory);
    glfwSetWindowUserPointer(window, &stateHandler);

    // Set key callback
    glfwSetKeyCallback(window, keyCallback);

     // Loop until window should close
    for(unsigned int frame = 0; !glfwWindowShouldClose(window); frame++) {
        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Update
        if(!stateHandler.update()) {
            break;
        }

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }


    glfwTerminate();
    return 0;
}