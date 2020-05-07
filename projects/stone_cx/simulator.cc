// Standard C++ includes
#include <numeric>
#include <random>
#include <vector>

// Standard C includes
#include <cmath>
#include <cstdlib>

// OpenCV includes
#include <opencv2/opencv.hpp>

// GeNN user project includes
#include "analogueRecorder.h"
#include "sharedLibraryModel.h"

// Common includes
#include "common/logging.h"
#include "common/von_mises_distribution.h"

// Model includes
#include "parameters.h"
#include "spline.h"
#include "visualizationCommon.h"

using namespace BoBRobotics;
using namespace BoBRobotics::StoneCX;

int main()
{
    // Simulation rendering parameters
    const unsigned int pathImageSize = 1000;
    const unsigned int activityImageWidth = 500;
    const unsigned int activityImageHeight = 1000;

    const double preferredAngleTN2[] = { Parameters::pi / 4.0, -Parameters::pi / 4.0 };

    // Outbound path generation parameters
    const unsigned int numOutwardTimesteps = 1500;

    // Agent dynamics parameters
    const double pathLambda = 0.4;
    const double pathKappa = 100.0;

    const double agentDrag = 0.15;

    const double agentMinAcceleration = 0.0;
    const double agentMaxAcceleration = 0.15;
    const double agentM = 0.5;

    SharedLibraryModel<float> slm("", "stone_cx");
    slm.allocateMem();
    slm.initialize();
    slm.initializeSparse();

    cv::namedWindow("Path", cv::WINDOW_NORMAL);
    cv::resizeWindow("Path", pathImageSize, pathImageSize);
    cv::Mat pathImage(pathImageSize, pathImageSize, CV_8UC3, cv::Scalar::all(0));

    cv::namedWindow("Activity", cv::WINDOW_NORMAL);
    cv::resizeWindow("Activity", activityImageWidth, activityImageHeight);
    cv::moveWindow("Activity", pathImageSize, 0);
    cv::Mat activityImage(activityImageHeight, activityImageWidth, CV_8UC3, cv::Scalar::all(0));

    // Create Von Mises distribution to sample angular acceleration from
    std::array<uint32_t, std::mt19937::state_size> seedData;
    std::random_device seedSource;
    std::generate(seedData.begin(), seedData.end(),
                  [&seedSource](){ return seedSource(); });
    std::seed_seq seeds(std::begin(seedData), std::end(seedData));
    std::mt19937 gen(seeds);

    VonMisesDistribution<double> pathVonMises(0.0, pathKappa);

    // Create acceleration spline
    tk::spline accelerationSpline;
    {
        // Create vectors to hold the times at which linear acceleration
        // should change and it's values at those time
        const unsigned int numAccelerationChanges = numOutwardTimesteps / 50;
        std::vector<double> accelerationTime(numAccelerationChanges);
        std::vector<double> accelerationMagnitude(numAccelerationChanges);

        // Draw accelerations from real distribution
        std::uniform_real_distribution<double> acceleration(agentMinAcceleration,
                                                            agentMaxAcceleration);
        std::generate(accelerationMagnitude.begin(), accelerationMagnitude.end(),
                      [&gen, &acceleration](){ return acceleration(gen); });

        for(unsigned int i = 0; i < numAccelerationChanges; i++) {
            accelerationTime[i] = i * 50;
        }

        // Build spline from these
        accelerationSpline.set_points(accelerationTime, accelerationMagnitude);
    }

#ifdef RECORD_ELECTROPHYS
    GeNNUtils::AnalogueCSVRecorder<scalar> tn2Recorder("tn2.csv", rTN2, Parameters::numTN2, "TN2");
    GeNNUtils::AnalogueCSVRecorder<scalar> cl1Recorder("cl1.csv", rCL1, Parameters::numCL1, "CL1");
    GeNNUtils::AnalogueCSVRecorder<scalar> tb1Recorder("tb1.csv", rTB1, Parameters::numTB1, "TB1");
    GeNNUtils::AnalogueCSVRecorder<scalar> cpu4Recorder("cpu4.csv", rCPU4, Parameters::numCPU4, "CPU4");
    GeNNUtils::AnalogueCSVRecorder<scalar> cpu1Recorder("cpu1.csv", rCPU1, Parameters::numCPU1, "CPU1");
#endif  // RECORD_ELECTROPHYS

    // Simulate
    double omega = 0.0;
    double theta = 0.0;
    double xVelocity = 0.0;
    double yVelocity = 0.0;
    double xPosition = 0.0;
    double yPosition = 0.0;
    float *headingAngleTL = slm.getScalar<float>("headingAngleTL");
    float *speedTN2 = slm.getArray<float>("speedTN2");
    float *rTL = slm.getArray<float>("rTL");
    const float *rCL1 = slm.getArray<float>("rCL1");
    const float *rTB1 = slm.getArray<float>("rTB1");
    const float *rTN2 = slm.getArray<float>("rTN2");
    const float *iCPU4 = slm.getArray<float>("iCPU4");
    const float *rCPU4 = slm.getArray<float>("rCPU4");
    const float *rPontine = slm.getArray<float>("rPontine");
    const float *rCPU1 = slm.getArray<float>("rCPU1");
    for(unsigned int i = 0;; i++) {
        // Project velocity onto each TN2 cell's preferred angle and use as speed input
        for(unsigned int j = 0; j < Parameters::numTN2; j++) {
            speedTN2[j] = (sin(theta + preferredAngleTN2[j]) * xVelocity) +
                (cos(theta + preferredAngleTN2[j]) * yVelocity);
        }

        // Push inputs to device
        slm.pushVarToDevice("TN2", "speed");

        // Update TL input
        *headingAngleTL = theta;

        // Step network
        slm.stepTime();

        // Pull outputs from device
        slm.pullVarFromDevice("TL", "r");
        slm.pullVarFromDevice("TN2", "r");
        slm.pullVarFromDevice("CL1", "r");
        slm.pullVarFromDevice("TB1", "r");
        slm.pullVarFromDevice("CPU4", "i");
        slm.pullVarFromDevice("CPU4", "r");
        slm.pullVarFromDevice("CPU1", "r");
        slm.pullVarFromDevice("Pontine", "r");

#ifdef RECORD_ELECTROPHYS
        tn2Recorder.record(i);
        cl1Recorder.record(i);
        tb1Recorder.record(i);
        cpu4Recorder.record(i);
        cpu1Recorder.record(i);
#endif  // RECORD_ELECTROPHYS

        // Visualize network activity
        visualize(activityImage, rTL, rCL1, rTB1, rTN2, rCPU4, rPontine, rCPU1);

        // If we are on outbound segment of route
        const bool outbound = (i < numOutwardTimesteps);
        double a = 0.0;
        if(outbound) {
            // Update angular velocity
            omega = (pathLambda * omega) + pathVonMises(gen);

            // Read linear acceleration off spline
            a = accelerationSpline((double)i);

            if(i == (numOutwardTimesteps - 1)) {
                LOGI << "Max CPU4 level r=" << *std::max_element(&rCPU4[0], &rCPU4[Parameters::numCPU4]) << ", i=" << *std::max_element(&iCPU4[0], &iCPU4[Parameters::numCPU4]);
            }
        }
        // Otherwise we're path integrating home
        else {
            // Sum left and right motor activity
            const float leftMotor = std::accumulate(&rCPU1[0], &rCPU1[8], 0.0f);
            const float rightMotor = std::accumulate(&rCPU1[8], &rCPU1[16], 0.0f);

            // Use difference between left and right to calculate angular velocity
            omega = -agentM * (rightMotor - leftMotor);

            // Use fixed acceleration
            a = 0.1;
        }

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

        // Draw agent position (centring so origin is in centre of path image)
        const cv::Point p((pathImageSize / 2) + (int)xPosition, (pathImageSize / 2) + (int)yPosition);
        cv::line(pathImage, p, p,
                 outbound ? CV_RGB(0xFF, 0, 0) : CV_RGB(0, 0xFF, 0));

        // Show output image
        cv::imshow("Path", pathImage);
        cv::imshow("Activity", activityImage);
        if(cv::waitKey(1) == 27) {
            break;
        }
    }
    return 0;
}
