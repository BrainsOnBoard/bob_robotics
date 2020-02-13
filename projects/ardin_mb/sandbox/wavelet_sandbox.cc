
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <unistd.h>
#include <string>
int main() 
{
    
    cv::Mat img = cv::imread("home/stefan/Documents/BoB_Code/bob_robotics/projects/ardin_mb/sandbox/img.jpg", CV_LOAD_IMAGE_UNCHANGED);
    cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);
    cv::imshow("Display window",img);
    sleep(2);
    std::cout << "Hello, World!";
    return 0;
}