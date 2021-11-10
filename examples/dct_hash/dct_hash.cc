#include "common/path.h"
#include "imgproc/dct_hash.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace BoBRobotics::Path;
using namespace BoBRobotics::ImgProc;

int
bobMain(int, char **)
{
    const auto imagePath1 = getProgramDirectory() / "library1.jpg";
    const auto imagePath2 = getProgramDirectory() / "library2.jpg";
    cv::Mat img1 = imread(imagePath1.str(), cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = imread(imagePath2.str(), cv::IMREAD_GRAYSCALE);

    // convert to calculate dct
    img1.convertTo(img1, CV_32F, 1.0 / 255);
    img2.convertTo(img2, CV_32F, 1.0 / 255);

    cv::imshow("Display window1", img1);
    cv::imshow("Display window2", img2);

    auto hash1 = DCTHash::computeHash(img1);
    auto hash2 = DCTHash::computeHash(img2);

    int distance = DCTHash::distance(hash1, hash2);
    std::cout << "the distance between the 2 images is " << distance << std::endl;

    cv::waitKey(5000); // Wait for 5 sec in the window

    return 0;
}
