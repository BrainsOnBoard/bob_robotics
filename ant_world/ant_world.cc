// Standard C++ includes
#include <bitset>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <tuple>
#include <vector>

// Standard C includes
#include <cmath>

// OpenCV includes
#include <opencv2/opencv.hpp>

// OpenGL includes
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// Common includes
#include "../common/timer.h"

// Libantworld includes
#include "common.h"
#include "renderer.h"
#include "route_ardin.h"
#include "snapshot_processor_ardin.h"

// Antworld includes
#include "mb_memory.h"
#include "perfect_memory.h"
#include "parameters.h"

using namespace GeNN_Robotics;

//----------------------------------------------------------------------------
// Anonymous namespace
//----------------------------------------------------------------------------
namespace
{
enum class State
{
    Training,
    Testing,
    RandomWalk,
    SpinningTrain,
    SpinningTest,
    Idle,
};

// Enumeration of keys
enum Key
{
    KeyLeft,
    KeyRight,
    KeyUp,
    KeyDown,
    KeyTrainSnapshot,
    KeyTestSnapshot,
    KeySpin,
    KeyReset,
    KeyMax
};

// Bitset used for passing which keys have been pressed between key callback and render loop
typedef std::bitset<KeyMax> KeyBitset;

//----------------------------------------------------------------------------
void keyCallback(GLFWwindow *window, int key, int, int action, int)
{
    // If action isn't a press or a release, do nothing
    if(action != GLFW_PRESS && action != GLFW_RELEASE) {
        return;
    }

    // Determine what state key bit should be set to
    const bool newKeyState = (action == GLFW_PRESS);

    // Extract key bitset from window's user pointer
    KeyBitset *keybits = (KeyBitset*)glfwGetWindowUserPointer(window);

    // Apply new key state to bits of key bits
    switch(key) {
        case GLFW_KEY_LEFT:
            keybits->set(KeyLeft, newKeyState);
            break;

        case GLFW_KEY_RIGHT:
            keybits->set(KeyRight, newKeyState);
            break;

        case GLFW_KEY_UP:
            keybits->set(KeyUp, newKeyState);
            break;

        case GLFW_KEY_DOWN:
            keybits->set(KeyDown, newKeyState);
            break;

        case GLFW_KEY_SPACE:
            keybits->set(KeyTrainSnapshot, newKeyState);
            break;

        case GLFW_KEY_ENTER:
            keybits->set(KeyTestSnapshot, newKeyState);
            break;

        case GLFW_KEY_R:
            keybits->set(KeyReset, newKeyState);
            break;

        case GLFW_KEY_S:
            keybits->set(KeySpin, newKeyState);
            break;
    }
}
//----------------------------------------------------------------------------
void handleGLFWError(int errorNumber, const char *message)
{
    std::cerr << "GLFW error number:" << errorNumber << ", message:" << message << std::endl;
}
}   // anonymous namespace
//----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    std::mt19937 gen;

    // Set GLFW error callback
    glfwSetErrorCallback(handleGLFWError);

    // Initialize the library
    if(!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    // Prevent window being resized
    glfwWindowHint(GLFW_RESIZABLE, false);

    // Create a windowed mode window and its OpenGL context
    GLFWwindow *window = glfwCreateWindow(Parameters::displayRenderWidth, Parameters::displayRenderHeight + Parameters::displayRenderWidth + 10,
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
    glfwSwapInterval(2);

    // Set clear colour to match matlab and enable depth test
    glClearColor(0.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glLineWidth(4.0);
    glPointSize(4.0);

    // Create key bitset and setc it as window user pointer
    KeyBitset keybits;
    glfwSetWindowUserPointer(window, &keybits);

    // Set key callback
    glfwSetKeyCallback(window, keyCallback);

    // Create renderer
    Renderer renderer("../libantworld/world5000_gray.bin", Parameters::worldColour, Parameters::groundColour,
                      Parameters::displayRenderWidth, Parameters::displayRenderHeight);

    // Create route object and load route file specified by command line
    RouteArdin route(0.2f, 800);
    if(argc > 1) {
        route.load(argv[1]);
    }

    // Create memory
    MBMemory memory;
    //PerfectMemory memory;

    // Host OpenCV array to hold pixels read from screen
    cv::Mat snapshot(Parameters::displayRenderHeight, Parameters::displayRenderWidth, CV_8UC3);

    // Create snapshot processor to perform image processing on snapshot
    SnapshotProcessorArdin snapshotProcessor(Parameters::displayScale,
                                             Parameters::intermediateSnapshotWidth, Parameters::intermediateSnapshotHeight,
                                             Parameters::inputWidth, Parameters::inputHeight);

    // Initialize ant position
    float antX = 5.0f;
    float antY = 5.0f;
    float antHeading = 270.0f;
    if(route.size() > 0) {
        std::tie(antX, antY, antHeading) = route[0];
    }

    // If a route is loaded, start in training mode, otherwise idle
    State state = (route.size() > 0) ? State::Training : State::Idle;
    //State state = State::RandomWalk;

    size_t trainPoint = 0;
    size_t maxTestPoint = 0;

    unsigned int testingScan = 0;

    unsigned int numErrors = 0;
    unsigned int numTestSteps = 0;

    float bestHeading = 0.0f;
    unsigned int bestTestENSpikes = std::numeric_limits<unsigned int>::max();

    // Calculate scan parameters
    constexpr double halfScanAngle = Parameters::scanAngle / 2.0;
    constexpr unsigned int numScanSteps = (unsigned int)round(Parameters::scanAngle / Parameters::scanStep);
    constexpr unsigned int numSpinSteps = (unsigned int)round(Parameters::scanAngle / Parameters::spinStep);

    // When random walking, distribution of angles to turn by
    std::uniform_real_distribution<float> randomAngleOffset(-halfScanAngle, halfScanAngle);

    std::ofstream spin;

    std::future<std::tuple<unsigned int, unsigned int, unsigned int>> gennResult;
    while (!glfwWindowShouldClose(window)) {
        // If there is no valid result (GeNN process has never run), we are ready to take a snapshot
        bool readyForNextSnapshot = false;
        bool resultsAvailable = false;
        unsigned int numPNSpikes;
        unsigned int numKCSpikes;
        unsigned int numENSpikes;
        if(!gennResult.valid()) {
            readyForNextSnapshot = true;
        }
        // Otherwise if GeNN has run and the result is ready for us, s
        else if(gennResult.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            std::tie(numPNSpikes, numKCSpikes, numENSpikes) = gennResult.get();
            std::cout << "\t" << numPNSpikes << " PN spikes, " << numKCSpikes << " KC spikes, " << numENSpikes << " EN spikes" << std::endl;

            readyForNextSnapshot = true;
            resultsAvailable = true;
        }

        // Update heading and ant position based on keys
        bool trainSnapshot = false;
        bool testSnapshot = false;
        if(keybits.test(KeyLeft)) {
            antHeading -= Parameters::antTurnSpeed;
        }
        if(keybits.test(KeyRight)) {
            antHeading += Parameters::antTurnSpeed;
        }
        if(keybits.test(KeyUp)) {
            antX += Parameters::antMoveSpeed * sin(antHeading * degreesToRadians);
            antY += Parameters::antMoveSpeed * cos(antHeading * degreesToRadians);
        }
        if(keybits.test(KeyDown)) {
            antX -= Parameters::antMoveSpeed * sin(antHeading * degreesToRadians);
            antY -= Parameters::antMoveSpeed * cos(antHeading * degreesToRadians);
        }
        if(keybits.test(KeyReset)) {
            if(route.size() > 0) {
                std::tie(antX, antY, antHeading) = route[0];
            }
            else {
                antX = 5.0f;
                antY = 5.0f;
                antHeading = 270.0f;
            }
        }
        if(keybits.test(KeySpin) && state == State::Idle) {
            trainSnapshot = true;
            state = State::SpinningTrain;
        }

        // If GeNN is ready to handle next snapshot, trigger snapshots if keys are pressed
        if(readyForNextSnapshot && keybits.test(KeyTrainSnapshot)) {
            trainSnapshot = true;
        }
        if(readyForNextSnapshot && keybits.test(KeyTestSnapshot)) {
            testSnapshot = true;
        }

        // If we're training
        if(state == State::Training) {
            // If results from previous training snapshot are available, mark them on route
            if(resultsAvailable) {
                route.setWaypointFamiliarity(trainPoint - 1,
                                             (double)numENSpikes / 20.0);
            }

            // If GeNN is free to process next snapshot
            if(readyForNextSnapshot) {
                // If GeNN isn't training and we have more route points to train
                if(trainPoint < route.size()) {
                    // Snap ant to next snapshot point
                    std::tie(antX, antY, antHeading) = route[trainPoint];

                    // Update window title
                    std::string windowTitle = "Ant World - Training snaphot " + std::to_string(trainPoint) + "/" + std::to_string(route.size());
                    glfwSetWindowTitle(window, windowTitle.c_str());

                    // Set flag to train this snapshot
                    trainSnapshot = true;

                    // Go onto next training point
                    trainPoint++;
                }
                // Otherwise, if we've reached end of route
                else {
                    std::cout << "Training complete (" << route.size() << " snapshots)" << std::endl;

                    // Go to testing state
                    state = State::Testing;

                    // Snap ant back to start of route, facing in starting scan direction
                    std::tie(antX, antY, antHeading) = route[0];
                    antHeading -= halfScanAngle;

                    // Add initial replay point to route
                    route.addPoint(antX, antY, false);

                    // Reset scan
                    testingScan = 0;
                    bestTestENSpikes = std::numeric_limits<unsigned int>::max();

                    // Take snapshot
                    testSnapshot = true;
                }
            }
        }
        // Otherwise, if we're testing
        else if(state == State::Testing) {
            if(resultsAvailable) {
                // If this is an improvement on previous best spike count
                if(numENSpikes < bestTestENSpikes) {
                    bestHeading = antHeading;
                    bestTestENSpikes = numENSpikes;

                    std::cout << "\tUpdated result: " << bestHeading << " is most familiar heading with " << bestTestENSpikes << " spikes" << std::endl;
                }

                // Update window title
                std::string windowTitle = "Ant World - Testing with " + std::to_string(numErrors) + " errors";
                glfwSetWindowTitle(window, windowTitle.c_str());

                // Go onto next scan
                testingScan++;

                // If scan isn't complete
                if(testingScan < numScanSteps) {
                    // Scan right
                    antHeading += Parameters::scanStep;

                    // Take test snapshot
                    testSnapshot = true;
                }
                else {
                    std::cout << "Scan complete: " << bestHeading << " is most familiar heading with " << bestTestENSpikes << " spikes" << std::endl;

                    // Snap ant to it's best heading
                    antHeading = bestHeading;

                    // Increment step count
                    numTestSteps++;

                    // Move ant forward by snapshot distance
                    antX += Parameters::snapshotDistance * sin(antHeading * degreesToRadians);
                    antY += Parameters::snapshotDistance * cos(antHeading * degreesToRadians);

                    // If we've reached destination
                    if(route.atDestination(antX, antY, Parameters::errorDistance)) {
                        std::cerr << "Destination reached in " << numTestSteps << " steps with " << numErrors << " errors" << std::endl;

                        // Reset state to idle
                        state = State::Idle;

                        // Add final point to route
                        route.addPoint(antX, antY, false);

                        // Stop
                        return 0;
                    }
                    // Otherwise, if we've
                    else if(numTestSteps >= Parameters::testStepLimit) {
                        std::cerr << "Failed to find destination after " << numTestSteps << " steps and " << numErrors << " errors" << std::endl;

                        // Stop
                        return 0;
                    }
                    // Otherwise
                    else {
                        // Calculate distance to route
                        float distanceToRoute;
                        size_t nearestRouteWaypoint;
                        std::tie(distanceToRoute, nearestRouteWaypoint) = route.getDistanceToRoute(antX, antY);
                        std::cout << "\tDistance to route: " << distanceToRoute * 100.0f << "cm" << std::endl;

                        // If we are further away than error threshold
                        if(distanceToRoute > Parameters::errorDistance) {
                            // If we have previously reached further down route than nearest point
                            // the furthest point reached is our 'best' waypoint otherwise it's the nearest waypoint
                            const size_t bestWaypoint = (nearestRouteWaypoint < maxTestPoint) ? maxTestPoint : nearestRouteWaypoint;

                            // Snap ant to the waypoint after this (clamping to size of route)
                            const size_t snapWaypoint = std::min(bestWaypoint + 1, route.size() - 1);
                            std::tie(antX, antY, antHeading) = route[snapWaypoint];

                            // Update maximum test point reached
                            maxTestPoint = std::max(maxTestPoint, snapWaypoint);

                            // Add error point to route
                            route.addPoint(antX, antY, true);

                            // Increment error counter
                            numErrors++;
                        }
                        // Otherwise, update maximum test point reached and add 'correct' point to route
                        else {
                            maxTestPoint = std::max(maxTestPoint, nearestRouteWaypoint);
                            route.addPoint(antX, antY, false);
                        }

                        // Reset scan
                        antHeading -= halfScanAngle;
                        testingScan = 0;
                        bestTestENSpikes = std::numeric_limits<unsigned int>::max();

                        // Take snapshot
                        testSnapshot = true;
                    }
                }
            }
        }
        else if(state == State::RandomWalk) {
            // Pick random heading
            antHeading += randomAngleOffset(gen);

            // Move ant forward by snapshot distance
            antX += Parameters::snapshotDistance * sin(antHeading * degreesToRadians);
            antY += Parameters::snapshotDistance * cos(antHeading * degreesToRadians);

            // If we've reached destination
            if(route.atDestination(antX, antY, Parameters::errorDistance)) {
                std::cout << "Destination reached with " << numErrors << " errors" << std::endl;

                // Reset state to idle
                state = State::Idle;

                // Add final point to route
                route.addPoint(antX, antY, false);
            }
            // Otherwise
            else {
                // Calculate distance to route
                float distanceToRoute;
                size_t nearestRouteWaypoint;
                std::tie(distanceToRoute, nearestRouteWaypoint) = route.getDistanceToRoute(antX, antY);

                // If we are further away than error threshold
                if(distanceToRoute > Parameters::errorDistance) {
                    // Snap ant to next snapshot position
                    // **HACK** this is dubious but looks very much like what the original model was doing in figure 1i
                    std::tie(antX, antY, antHeading) = route[nearestRouteWaypoint + 1];

                    // Add error point to route
                    route.addPoint(antX, antY, true);

                    // Increment error counter
                    numErrors++;
                }
                // Otherwise add 'correct' point to route
                else {
                    route.addPoint(antX, antY, false);
                }
            }
        }
        if(state == State::SpinningTrain) {
            if(resultsAvailable) {
                spin.open("spin.csv");

                // Start testing scan
                state = State::SpinningTest;
                antHeading -= halfScanAngle;
                testingScan = 0;
                testSnapshot = true;
            }
        }
        else if(state == State::SpinningTest) {
            if(resultsAvailable) {
                   // Write heading and number of spikes to file
                spin << antHeading << "," << numENSpikes << std::endl;

                // Go onto next scan
                testingScan++;

                // If scan isn't complete
                if(testingScan < numSpinSteps) {
                    // Scan right
                    antHeading += Parameters::spinStep;

                    // Take test snapshot
                    testSnapshot = true;
                }
                else {
                    spin.close();

                    state = State::Idle;
                }
            }
        }

        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render top down and ants eye view
        renderer.render(antX, antY, antHeading);

        // Render route
        route.render(antX, antY, antHeading);
        
        // Swap front and back buffers
        glfwSwapBuffers(window);

        // If we should take a snapshot
        if(trainSnapshot || testSnapshot) {
            Timer<> timer("\tSnapshot generation:");

            std::cout << "Snapshot at (" << antX << "," << antY << "," << antHeading << ")" << std::endl;

            // Read pixels from framebuffer
            // **TODO** it should be theoretically possible to go directly from frame buffer to GpuMat
            glReadPixels(0, Parameters::displayRenderWidth + 10, Parameters::displayRenderWidth, Parameters::displayRenderHeight,
                         GL_BGR, GL_UNSIGNED_BYTE, snapshot.data);

            // Process snapshot
            snapshotProcessor.process(snapshot);

            // Present to memory
            gennResult = memory.present(snapshotProcessor.getFinalSnapshotFloat(), trainSnapshot);

        }

        // Poll for and process events
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}