#include "visualizationCommon.h"

// Standard C includes
#include <cmath>

// OpenCV includes
#include <opencv2/opencv.hpp>

// GeNN generated code includes
#include "stone_cx_CODE/definitions.h"

// Model includes
#include "parameters.h"

//---------------------------------------------------------------------------
// Anonymous namespace
//---------------------------------------------------------------------------
namespace
{

template<typename F>
void drawNeuronActivity(scalar activity, const cv::Point &position, F getColourFn, cv::Mat &image)
{
    // Convert activity to a 8-bit level
    const unsigned char gray = (unsigned char)(255.0f * std::min(1.0f, std::max(0.0f, activity)));

    // Draw rectangle of this colour
    cv::rectangle(image, position, position + cv::Point(25, 25), getColourFn(gray), cv::FILLED);
}

template<typename F>
void drawPopulationActivity(scalar *popActivity, int popSize, const char *popName,
                            const cv::Point &position, F getColourFn, cv::Mat &image, int numColumns=0)
{
    // If (invalid) default number of columns is specified, use popsize
    if(numColumns == 0) {
        numColumns = popSize;
    }

    // Loop through each neuron in population
    for(int i = 0; i < popSize; i++) {
        // Calculate coordinate in terms of rows and columns
        auto coord = std::div(i, numColumns);
        cv::Point offset(coord.rem * 27, coord.quot * 27);

        // Draw neuron activity
        drawNeuronActivity(popActivity[i], position + offset, getColourFn, image);
    }

    // Label population
    const int numRows = (int)ceil((double)popSize / (double)numColumns);
    cv::putText(image, popName, position + cv::Point(0, 17 + (27 * numRows)),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0xFF, 0xFF, 0xFF));
}

cv::Scalar getReds(unsigned char gray)
{
    return CV_RGB(gray, 0, 0);
}

cv::Scalar getGreens(unsigned char gray)
{
    return CV_RGB(0, gray, 0);
}

cv::Scalar getBlues(unsigned char gray)
{
    return CV_RGB(0, 0, gray);
}
}   // Anonymous namespace

using namespace BoBRobotics::StoneCX;

void visualize(cv::Mat &activityImage)
{
    // Draw compass system activity
    drawPopulationActivity(rTL, Parameters::numTL, "TL", cv::Point(10, 10),
                           getReds, activityImage, 8);
    drawPopulationActivity(rCL1, Parameters::numCL1, "CL1", cv::Point(10, 110),
                           getReds, activityImage, 8);
    drawPopulationActivity(rTB1, Parameters::numTB1, "TB1", cv::Point(10, 210),
                           getReds, activityImage);

    drawPopulationActivity(rTN2, Parameters::numTN2, "TN2", cv::Point(300, 310),
                           getBlues, activityImage, 1);

    drawPopulationActivity(rCPU4, Parameters::numCPU4, "CPU4", cv::Point(10, 310),
                           getGreens, activityImage, 8);
    drawPopulationActivity(rPontine, Parameters::numPontine, "Pontine", cv::Point(10, 410),
                           getGreens, activityImage, 8);
    drawPopulationActivity(rCPU1, Parameters::numCPU1, "CPU1", cv::Point(10, 510),
                           getGreens, activityImage, 8);
}