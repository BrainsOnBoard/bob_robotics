// Standard C++ includes
#include <atomic>
#include <chrono>
#include <fstream>
#include <thread>

// Common includes
#include "../common/joystick.h"
#include "../common/lm9ds1_imu.h"
#include "../common/timer.h"
#include "../robots/motor_i2c.h"

// GeNN generated code includes
#include "stone_cx_CODE/definitions.h"

// Model includes
#include "parameters.h"
#include "robotCommon.h"
#include "robotParameters.h"
#include "simulatorCommon.h"

using namespace GeNNRobotics;

//---------------------------------------------------------------------------
// Anonymous namespace
//---------------------------------------------------------------------------
namespace
{
void imuThreadFunc(std::atomic<bool> &shouldQuit, std::atomic<float> &heading, unsigned int &numSamples)
{
    // Create IMU interface
    LM9DS1 imu;
    
    // Initialise IMU magnetometer
    LM9DS1::MagnetoSettings magSettings;
    imu.initMagneto(magSettings);
    
    // While quit signal isn't set
    for(numSamples = 0; !shouldQuit; numSamples++) {
        // Wait for magneto to become available
        while(!imu.isMagnetoAvailable()){
        }
        
        // Read magneto
        float magnetoData[3];
        imu.readMagneto(magnetoData);
            
        // Calculate heading angle from magneto data and set atomic value
        heading = atan2(magnetoData[0], magnetoData[2]);
    }
}
}   // Anonymous namespace


int main(int argc, char *argv[])
{
    const float velocityScale = 1.0f / 10.0f;
    
    // Create joystick interface
    Joystick joystick;
    
    // Create motor interface
    Robots::MotorI2C motor;
    
    // Initialise GeNN
    allocateMem();
    initialize();

    //---------------------------------------------------------------------------
    // Initialize neuron parameters
    //---------------------------------------------------------------------------
    // TL
    for(unsigned int i = 0; i < 8; i++) {
        preferredAngleTL[i] = preferredAngleTL[8 + i] = (Parameters::pi / 4.0) * (double)i;
    }

    //---------------------------------------------------------------------------
    // Build connectivity
    //---------------------------------------------------------------------------
    buildConnectivity();
    
    initstone_cx();
    
    // Atomic flag for quitting child threads
    std::atomic<bool> shouldQuit{false};
    
    // Create thread to read from IMU
    unsigned int numIMUSamples = 0;
    std::atomic<float> imuHeading{0.0f};
    std::thread imuThread(&imuThreadFunc, 
                          std::ref(shouldQuit), std::ref(imuHeading), std::ref(numIMUSamples));
    
#ifdef RECORD_SENSORS
    std::ofstream data("data.csv");
#endif
    // Loop until second joystick button is pressed
    bool outbound = true;
    unsigned int numTicks = 0;
    unsigned int numOverflowTicks = 0;
    int64_t totalMicroseconds = 0;
    for(;; numTicks++) {
        // Record time at start of tick
        const auto tickStartTime = std::chrono::high_resolution_clock::now();
        
        // Read from joystick
        joystick.read();
        
        // Stop if 2nd button is pressed
        if(joystick.isButtonDown(1)) {
            break;
        }

        // Update heading from IMU
        headingAngleTL = imuHeading;
        if(headingAngleTL < 0.0) {
            headingAngleTL = (2.0 * Parameters::pi) + headingAngleTL;
        }

        // Calculate dead reckoning speed from motor
        const float speed =  (motor.getLeft() + motor.getRight()) * velocityScale;
        speedTN2[Parameters::HemisphereLeft] = speedTN2[Parameters::HemisphereRight] = speed;

#ifdef RECORD_SENSORS
        data << imuHeading << ", " << speed << std::endl;
#endif
        // Step network
        stepTimeCPU();
        
        // If we are going outbound
        if(outbound) {
            // Use joystick to drive motor
            joystick.drive(motor, RobotParameters::joystickDeadzone);
            
            // If first button is pressed switch to returning home
            if(joystick.isButtonDown(0)) {
                std::cout << "Max CPU4 level r=" << *std::max_element(&rCPU4[0], &rCPU4[Parameters::numCPU4]) << ", i=" << *std::max_element(&iCPU4[0], &iCPU4[Parameters::numCPU4]) << std::endl;
                std::cout << "Returning home!" << std::endl;
                outbound = false;
            }

        }
        // Otherwise we're returning home so use CPU1 neurons to drive motor
        else {
            driveMotorFromCPU1(motor, (numTicks % 100) == 0);
        }
        
        // Record time at end of tick
        const auto tickEndTime = std::chrono::high_resolution_clock::now();
        
        // Calculate tick duration (in microseconds)
        const int64_t tickMicroseconds = std::chrono::duration_cast<chrono::microseconds>(tickEndTime - tickStartTime).count();
        
        // Add to total
        totalMicroseconds += tickMicroseconds;
        
        // If there is time left in tick, sleep for remainder
        if(tickMicroseconds < RobotParameters::targetTickMicroseconds) {
            std::this_thread::sleep_for(std::chrono::microseconds(RobotParameters::targetTickMicroseconds - tickMicroseconds));
        }
        // Otherwise, increment overflow counter
        else {
            numOverflowTicks++;
        }
    }
    
    // Set quit flag and wait for child threads to complete
    shouldQuit = true;
    imuThread.join();
    
    // Show stats
    std::cout << numOverflowTicks << "/" << numTicks << " ticks overflowed, mean tick time: " << (double)totalMicroseconds / (double)numTicks << "uS, ";
    std::cout << "IMU samples: " << numIMUSamples << ", ";
    
    // Stop motor
    motor.tank(0.0f, 0.0f);
    
    // Exit
    return EXIT_SUCCESS;
}
