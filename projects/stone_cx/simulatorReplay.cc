// **TODO**: This currently doesn't compile. It needs converting to GeNN 4.
// Standard C++ includes
#include <fstream>
#include <numeric>
#include <random>
#include <sstream>
#include <vector>

// Standard C includes
#include <cmath>
#include <cstdlib>

// OpenCV includes
#include <opencv2/opencv.hpp>

// Common includes
#include "common/logging.h"
#include "genn_utils/analogue_csv_recorder.h"

// GeNN generated code includes
#include "stone_cx_CODE/definitions.h"

// Model includes
#include "parameters.h"
#include "visualizationCommon.h"

using namespace BoBRobotics;
using namespace BoBRobotics::StoneCX;

namespace
{
double readDoubleField(std::istringstream &lineStream)
{
    std::string field;
    std::getline(lineStream, field, ',');
    return std::stod(field);
}
}

int main()
{
    // Simulation rendering parameters
    const unsigned int pathImageSize = 1000;
    const unsigned int activityImageWidth = 500;
    const unsigned int activityImageHeight = 1000;

    const double preferredAngleTN2[] = { Parameters::pi / 4.0, -Parameters::pi / 4.0 };

    const double agentDrag = 0.15;

    const double agentM = 0.5;

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

    cv::namedWindow("Path", cv::WINDOW_NORMAL);
    cv::resizeWindow("Path", pathImageSize, pathImageSize);
    cv::Mat pathImage(pathImageSize, pathImageSize, CV_8UC3, cv::Scalar::all(0));

    cv::namedWindow("Activity", cv::WINDOW_NORMAL);
    cv::resizeWindow("Activity", activityImageWidth, activityImageHeight);
    cv::moveWindow("Activity", pathImageSize, 0);
    cv::Mat activityImage(activityImageHeight, activityImageWidth, CV_8UC3, cv::Scalar::all(0));

#ifdef RECORD_ELECTROPHYS
    GeNNUtils::AnalogueCSVRecorder<scalar> tn2Recorder("tn2.csv", rTN2, Parameters::numTN2, "TN2");
    GeNNUtils::AnalogueCSVRecorder<scalar> cl1Recorder("cl1.csv", rCL1, Parameters::numCL1, "CL1");
    GeNNUtils::AnalogueCSVRecorder<scalar> tb1Recorder("tb1.csv", rTB1, Parameters::numTB1, "TB1");
    GeNNUtils::AnalogueCSVRecorder<scalar> cpu4Recorder("cpu4.csv", rCPU4, Parameters::numCPU4, "CPU4");
    GeNNUtils::AnalogueCSVRecorder<scalar> cpu1Recorder("cpu1.csv", rCPU1, Parameters::numCPU1, "CPU1");
#endif  // RECORD_ELECTROPHYS

    std::ifstream replayData("data.csv");

    // Simulate
    double theta = 0.0;
    double xVelocity = 0.0;
    double yVelocity = 0.0;
    double xPosition = 0.0;
    double yPosition = 0.0;
    double xStartPosition = 0.0;
    double yStartPosition = 0.0;
    bool first = true;
    for(unsigned int i = 0;; i++) {
        // Project velocity onto each TN2 cell's preferred angle and use as speed input
        for(unsigned int j = 0; j < Parameters::numTN2; j++) {
            speedTN2[j] = (sin(theta + preferredAngleTN2[j]) * xVelocity) +
                (cos(theta + preferredAngleTN2[j]) * yVelocity);
        }

        // Update TL input
        headingAngleTL = theta;

        // Step network
        stepTimeCPU();

#ifdef RECORD_ELECTROPHYS
        tn2Recorder.record(i);
        cl1Recorder.record(i);
        tb1Recorder.record(i);
        cpu4Recorder.record(i);
        cpu1Recorder.record(i);
#endif  // RECORD_ELECTROPHYS

        // Visualize network activity
        visualize(activityImage);

        // If we there is more data to replay
        std::string line;
        const bool outbound = !std::getline(replayData, line).fail();
        if(outbound) {
            // Read position from file
            std::istringstream lineStream(line);
            xPosition = readDoubleField(lineStream);
            yPosition = readDoubleField(lineStream);
            readDoubleField(lineStream);

            // If this is the first position, update start position so drawing can be centred
            if(first) {
                xStartPosition = xPosition;
                yStartPosition = yPosition;
                first = false;
            }

            // Read velocity
            xVelocity = 6.0f * readDoubleField(lineStream);
            yVelocity = 6.0f * readDoubleField(lineStream);
            readDoubleField(lineStream);

            // Read orientation, flipping and converting from (-pi,pi) to (0, 2pi)
            readDoubleField(lineStream);
            readDoubleField(lineStream);
            theta = -readDoubleField(lineStream);
            if(theta < 0.0) {
                theta = (2.0 * Parameters::pi) + theta;
            }

            // If this is the end of outbound data, zero velocity (as simulated dynamics don't match well) and display CPU4 high-water mark
            if(replayData.eof()) {
                xVelocity = 0.0;
                yVelocity = 0.0;
                LOGI << "Max CPU4 level r=" << *std::max_element(&rCPU4[0], &rCPU4[Parameters::numCPU4]) << ", i=" << *std::max_element(&iCPU4[0], &iCPU4[Parameters::numCPU4]);
            }
        }
        // Otherwise we're path integrating home
        else {
            // Sum left and right motor activity
            const scalar leftMotor = std::accumulate(&rCPU1[0], &rCPU1[8], 0.0f);
            const scalar rightMotor = std::accumulate(&rCPU1[8], &rCPU1[16], 0.0f);

            // Use difference between left and right to calculate angular velocity
            const double omega = -agentM * (rightMotor - leftMotor);

            // Use fixed acceleration
            const double a = 0.1;

            // Update heading
            theta += omega;

            // Update linear velocity
            // **NOTE** this comes from https://github.com/InsectRobotics/path-integration/blob/master/bee_simulator.py#L77-L83 rather than the methods section
            xVelocity += sin(theta) * a;
            yVelocity += cos(theta) * a;
            xVelocity -= agentDrag * xVelocity;
            yVelocity -= agentDrag * yVelocity;

            // Update position
            xPosition += xVelocity;
            yPosition += yVelocity;
        }

        // Draw agent position (centring so origin is in centre of path image)
        const cv::Point p((pathImageSize / 2) + (int)(0.25 * (xPosition - xStartPosition)), (pathImageSize / 2) + (int)(0.25 * (yPosition - yStartPosition)));
        cv::line(pathImage, p, p,
                 outbound ? CV_RGB(0xFF, 0, 0) : CV_RGB(0, 0xFF, 0));

        // Show output image
        cv::imshow("Path", pathImage);
        cv::imshow("Activity", activityImage);
        cv::waitKey(1);
    }
    return 0;
}