#include "imgproc/dct_hash.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

int bobMain(int, char **)
{
	std::string image_path1 = "library1.jpg";
	std::string image_path2 = "library2.jpg";
    cv::Mat img1 = imread(image_path1, cv::IMREAD_GRAYSCALE);
	cv::Mat img2 = imread(image_path2, cv::IMREAD_GRAYSCALE);

	// convert to calculate dct 
	img1.convertTo(img1, CV_32F, 1.0/255);
	img2.convertTo(img2, CV_32F, 1.0/255);

	cv::imshow("Display window1", img1);
	cv::imshow("Display window2", img2);

	auto hash1 = DCTHash::computeHash(img1);
	auto hash2 = DCTHash::computeHash(img2);

	int distance = DCTHash::distance(hash1,hash2);
	std::cout << "the distance between the 2 images is " << distance << std::endl;
   
	int k = cv::waitKey(0); // Wait for a keystroke in the window
   
    return 0;
}
