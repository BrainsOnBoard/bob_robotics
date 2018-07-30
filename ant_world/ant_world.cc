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

// OpenCV includes
#include <opencv2/opencv.hpp>

// OpenGL includes
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// BoB robotics includes
#include "../common/timer.h"
#include "../third_party/units.h"
#include "../video/opengl.h"

// Libantworld includes
#include "common.h"
#include "renderer.h"
#include "route_ardin.h"
#include "snapshot_processor_ardin.h"

// Antworld includes
#include "mb_memory.h"
#include "perfect_memory.h"
#include "mb_params.h"
#include "sim_params.h"

using namespace BoBRobotics;
using namespace units::angle;
using namespace units::length;
using namespace units::literals;

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

    // Create key bitset and setc it as window user pointer
    KeyBitset keybits;
    glfwSetWindowUserPointer(window, &keybits);

    // Set key callback
    glfwSetKeyCallback(window, keyCallback);

    // Create renderer
    AntWorld::Renderer renderer;
    renderer.getWorld().load("../libantworld/world5000_gray.bin", SimParams::worldColour, SimParams::groundColour);

    // Create route object and load route file specified by command line
    AntWorld::RouteArdin route(0.2f, 800);
    if(argc > 1) {
        route.load(argv[1], true);
    }

    // Create memory
    MBMemory memory;
    //PerfectMemory memory;

    // Host OpenCV array to hold pixels read from screen
    cv::Mat snapshot(SimParams::displayRenderHeight, SimParams::displayRenderWidth, CV_8UC3);

    // Create input to read snapshots from screen
    Video::OpenGL input(0, SimParams::displayRenderWidth + 10,
                        SimParams::displayRenderWidth, SimParams::displayRenderHeight);

    // Create snapshot processor to perform image processing on snapshot
    AntWorld::SnapshotProcessorArdin snapshotProcessor(SimParams::displayScale,
                                                       SimParams::intermediateSnapshotWidth, SimParams::intermediateSnapshotHeight,
                                                       MBParams::inputWidth, MBParams::inputHeight);

    // Initialize ant position
    meter_t antX = 5.0_m;
    meter_t antY = 5.0_m;
    degree_t antHeading = 270.0_deg;
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

    degree_t bestHeading = 0.0_deg;
    unsigned int bestTestENSpikes = std::numeric_limits<unsigned int>::max();

    // Calculate scan parameters
    const degree_t halfScanAngle = SimParams::scanAngle / 2.0;
    const unsigned int numScanSteps = (unsigned int)units::math::round(SimParams::scanAngle / SimParams::scanStep);
    const unsigned int numSpinSteps = (unsigned int)units::math::round(SimParams::scanAngle / SimParams::spinStep);

    // When random walking, distribution of angles to turn by
    std::uniform_real_distribution<float> randomAngleOffset(-halfScanAngle.value(), halfScanAngle.value());

    std::ofstream spin;

    unsigned int numPNSpikes;
    unsigned int numKCSpikes;
    unsigned int numENSpikes;
    while (!glfwWindowShouldClose(window)) {
        // Update heading and ant position based on keys
        bool trainSnapshot = false;
        bool testSnapshot = false;
        if(keybits.test(KeyLeft)) {
            antHeading -= SimParams::antTurnStep;
        }
        if(keybits.test(KeyRight)) {
            antHeading += SimParams::antTurnStep;
        }
        if(keybits.test(KeyUp)) {
            antX += SimParams::antMoveStep * units::math::sin(antHeading);
            antY += SimParams::antMoveStep * units::math::cos(antHeading);
        }
        if(keybits.test(KeyDown)) {
            antX -= SimParams::antMoveStep * units::math::sin(antHeading);
            antY -= SimParams::antMoveStep * units::math::cos(antHeading);
        }
        if(keybits.test(KeyReset)) {
            if(route.size() > 0) {
                std::tie(antX, antY, antHeading) = route[0];
            }
            else {
                antX = 5.0_m;
                antY = 5.0_m;
                antHeading = 270.0_deg;
            }
        }
        if(keybits.test(KeySpin) && state == State::Idle) {
            trainSnapshot = true;
            state = State::SpinningTrain;
        }

        // Trigger snapshots if keys are pressed
        if(keybits.test(KeyTrainSnapshot)) {
            trainSnapshot = true;
        }
        if( keybits.test(KeyTestSnapshot)) {
            testSnapshot = true;
        }

        // If we're training
        if(state == State::Training) {
            // Mark results from previous training snapshot on route
            route.setWaypointFamiliarity(trainPoint - 1, (double)numENSpikes / 20.0);

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
        // Otherwise, if we're testing
        else if(state == State::Testing) {
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
                antHeading += SimParams::scanStep;

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
                antX += SimParams::snapshotDistance * units::math::sin(antHeading);
                antY += SimParams::snapshotDistance * units::math::cos(antHeading);

                // If we've reached destination
                if(route.atDestination(antX, antY, SimParams::errorDistance)) {
                    std::cerr << "Destination reached in " << numTestSteps << " steps with " << numErrors << " errors" << std::endl;

                    // Reset state to idle
                    state = State::Idle;

                    // Add final point to route
                    route.addPoint(antX, antY, false);

                    // Stop
                    return 0;
                }
                // Otherwise, if we've
                else if(numTestSteps >= SimParams::testStepLimit) {
                    std::cerr << "Failed to find destination after " << numTestSteps << " steps and " << numErrors << " errors" << std::endl;

                    // Stop
                    return 0;
                }
                // Otherwise
                else {
                    // Calculate distance to route
                    meter_t distanceToRoute;
                    size_t nearestRouteWaypoint;
                    std::tie(distanceToRoute, nearestRouteWaypoint) = route.getDistanceToRoute(antX, antY);
                    std::cout << "\tDistance to route: " << distanceToRoute * 100.0f << "cm" << std::endl;

                    // If we are further away than error threshold
                    if(distanceToRoute > SimParams::errorDistance) {
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
        else if(state == State::RandomWalk) {
            // Pick random heading
            antHeading += units::make_unit<degree_t>(randomAngleOffset(gen));

            // Move ant forward by snapshot distance
            antX += SimParams::snapshotDistance * units::math::sin(antHeading);
            antY += SimParams::snapshotDistance * units::math::cos(antHeading);

            // If we've reached destination
            if(route.atDestination(antX, antY, SimParams::errorDistance)) {
                std::cout << "Destination reached with " << numErrors << " errors" << std::endl;

                // Reset state to idle
                state = State::Idle;

                // Add final point to route
                route.addPoint(antX, antY, false);
            }
            // Otherwise
            else {
                // Calculate distance to route
                meter_t distanceToRoute;
                size_t nearestRouteWaypoint;
                std::tie(distanceToRoute, nearestRouteWaypoint) = route.getDistanceToRoute(antX, antY);

                // If we are further away than error threshold
                if(distanceToRoute > SimParams::errorDistance) {
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
            spin.open("spin.csv");

            // Start testing scan
            state = State::SpinningTest;
            antHeading -= halfScanAngle;
            testingScan = 0;
            testSnapshot = true;
        }
        else if(state == State::SpinningTest) {
            // Write heading and number of spikes to file
            spin << antHeading << "," << numENSpikes << std::endl;

            // Go onto next scan
            testingScan++;

            // If scan isn't complete
            if(testingScan < numSpinSteps) {
                // Scan right
                antHeading += SimParams::spinStep;

                // Take test snapshot
                testSnapshot = true;
            }
            else {
                spin.close();

                state = State::Idle;
            }
        }

        // Clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render top down and ants eye view
        renderer.renderPanoramicView(antX, antY, 0.01_m,
                                     antHeading, 0.0_deg, 0.0_deg,
                                     0, SimParams::displayRenderWidth + 10, SimParams::displayRenderWidth, SimParams::displayRenderHeight);
        renderer.renderTopDownView(0, 0, SimParams::displayRenderWidth, SimParams::displayRenderWidth);

        // Render route
        route.render(antX, antY, antHeading);
        
        // Swap front and back buffers
        glfwSwapBuffers(window);

        // If we should take a snapshot
        if(trainSnapshot || testSnapshot) {
            Timer<> timer("\tSnapshot generation:");

            std::cout << "Snapshot at (" << antX << "," << antY << "," << antHeading << ")" << std::endl;

            // Read snapshot
            input.readFrame(snapshot);

            // Process snapshot
            snapshotProcessor.process(snapshot);

            // Present to memory
            std::tie(numPNSpikes, numKCSpikes, numENSpikes) = memory.present(snapshotProcessor.getFinalSnapshotFloat(), trainSnapshot);
            std::cout << "\t" << numPNSpikes << " PN spikes, " << numKCSpikes << " KC spikes, " << numENSpikes << " EN spikes" << std::endl;
        }

        // Poll for and process events
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}