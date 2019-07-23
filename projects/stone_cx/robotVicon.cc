// Standard C++ includes
#include <chrono>
#include <numeric>

// Common includes
#include "common/logging.h"
#include "hid/joystick.h"
#include "genn_utils/analogue_csv_recorder.h"
#include "robots/tank.h"
#include "vicon/capture_control.h"
#include "vicon/udp.h"

// GeNN generated code includes
#include "stone_cx_CODE/definitions.h"

// Model includes
#include "parameters.h"
#include "robotCommon.h"
#include "robotParameters.h"

using namespace BoBRobotics;
using namespace BoBRobotics::StoneCX;
using namespace BoBRobotics::HID;
using namespace std::literals;

int main()
{
    const float speedScale = 5.0f;
    const double preferredAngleTN2[] = { Parameters::pi / 4.0, -Parameters::pi / 4.0 };

    // Create joystick interface
    Joystick joystick;

    // Create motor interface
    Robots::TANK_TYPE motor;

    // Create VICON UDP interface
    Vicon::UDPClient<Vicon::ObjectDataVelocity> vicon(51001);

    // Create VICON capture control interface
    Vicon::CaptureControl viconCaptureControl("192.168.1.100", 3003,
                                              "c:\\users\\ad374\\Desktop");
    // Initialise GeNN
    allocateMem();
    initialize();
    initializeSparse();

#ifdef RECORD_ELECTROPHYS
    GeNNUtils::AnalogueCSVRecorder<scalar> tn2Recorder("tn2.csv", rTN2, Parameters::numTN2, "TN2");
    GeNNUtils::AnalogueCSVRecorder<scalar> cl1Recorder("cl1.csv", rCL1, Parameters::numCL1, "CL1");
    GeNNUtils::AnalogueCSVRecorder<scalar> tb1Recorder("tb1.csv", rTB1, Parameters::numTB1, "TB1");
    GeNNUtils::AnalogueCSVRecorder<scalar> cpu4Recorder("cpu4.csv", rCPU4, Parameters::numCPU4, "CPU4");
    GeNNUtils::AnalogueCSVRecorder<scalar> cpu1Recorder("cpu1.csv", rCPU1, Parameters::numCPU1, "CPU1");
#endif  // RECORD_ELECTROPHYS

    // Start capture
    viconCaptureControl.startRecording("test");

    LOGI << "Start VICON frame:" << vicon.getObjectData().getFrameNumber();

    // Loop until second joystick button is pressed
    bool outbound = true;
    unsigned int numTicks = 0;
    unsigned int numOverflowTicks = 0;
    int64_t totalMicroseconds = 0;
    for(;; numTicks++) {
        // Record time at start of tick
        const auto tickStartTime = std::chrono::high_resolution_clock::now();

        // Read from joystick
        joystick.update();

        // Stop if 2nd button is pressed
        if(joystick.isDown(JButton::B)) {
            break;
        }

        // Read data from VICON system
        auto objectData = vicon.getObjectData();
        const auto &velocity = objectData.getVelocity();
        const auto &attitude = objectData.getAttitude();

        /*
         * Update TL input
         * **TODO**: We could update definitions.h etc. to use physical unit types,
         * and then this would all gel together nicely.
         */
        headingAngleTL = -attitude[2].value();
        if(headingAngleTL < 0.0) {
            headingAngleTL = (2.0 * Parameters::pi) + headingAngleTL;
        }

        // Project velocity onto each TN2 cell's preferred angle and use as speed input
        for(unsigned int j = 0; j < Parameters::numTN2; j++) {
            speedTN2[j] = (sin(headingAngleTL + preferredAngleTN2[j]) * speedScale * velocity[0].value()) +
                (cos(headingAngleTL + preferredAngleTN2[j]) * speedScale * velocity[1].value());
        }

        if(numTicks % 100 == 0) {
            LOGI <<  "Ticks:" << numTicks << ", Heading: " << headingAngleTL << ", Speed: (" << speedTN2[0] << ", " << speedTN2[1] << ")";
        }

        // Push inputs to device
        pushspeedTN2ToDevice();

        // Step network
        stepTime();

        // Pull outputs from device
        pullrCPU4FromDevice();
        pullrCPU1FromDevice();

#ifdef RECORD_ELECTROPHYS
        pullrTLFromDevice();
        pullrTN2FromDevice();
        pullrCL1FromDevice();
        pullrTB1FromDevice();

        tn2Recorder.record(numTicks);
        cl1Recorder.record(numTicks);
        tb1Recorder.record(numTicks);
        cpu4Recorder.record(numTicks);
        cpu1Recorder.record(numTicks);
#endif  // RECORD_ELECTROPHYS

        // If we are going outbound
        if(outbound) {
            // Use joystick to drive motor
            motor.drive(joystick, RobotParameters::joystickDeadzone);

            // If first button is pressed switch to returning home
            if(joystick.isDown(JButton::A)) {
                LOGI << "Max CPU4 level r=" << *std::max_element(&rCPU4[0], &rCPU4[Parameters::numCPU4]) << ", i=" << *std::max_element(&iCPU4[0], &iCPU4[Parameters::numCPU4]);
                LOGI << "Returning home!";
                LOGI << "Turn around VICON frame:" << objectData.getFrameNumber();
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
        const int64_t tickMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(tickEndTime - tickStartTime).count();

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

    // Show overflow stats
    LOGI << numOverflowTicks << "/" << numTicks << " ticks overflowed, mean tick time: " << (double)totalMicroseconds / (double)numTicks << "uS";

    // Stop motor
    motor.tank(0.0f, 0.0f);

    // Stop capture
    viconCaptureControl.stopRecording("test");

    // Exit
    return EXIT_SUCCESS;
}
