
#include "opencv2/imgproc/imgproc.hpp"
#include "third_party/wavelib/linuxshared/wavelet2d.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <unistd.h>

using namespace cv;
using namespace std;

std::vector<vector<double>>
Image2Array(Mat matImage)
{
    int rows = (int) matImage.rows;
    int cols = (int) matImage.cols;
    vector<vector<double>> vecImage(rows, vector<double>(cols));
    int k = 1;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            unsigned char temp;
            temp = ((uchar *) matImage.data + i * matImage.step)[j * matImage.elemSize() + k];
            vecImage[i][j] = (double) temp;
        }
    }
    return vecImage;
}

main(int argc, char **argv)
{
    // Read the image file
    Mat image = imread("/home/stefan/Documents/BoB_Code/bob_robotics/projects/ardin_mb/sandbox/img.jpg");

    // Check for failure
    if (image.empty()) {
        cout << "Could not open or find the image" << endl;
        cin.get(); //wait for any key press
        return -1;
    }

    string nm = "db2";
    int level = 3;
    vector<int> length;
    vector<double> coeffs, flag;
    vector<vector<double>> vectorImage = Image2Array(image);
    WAVELET2D_H::dwt_2d(vectorImage, level, nm,coeffs, flag, length);
    
    // Display stuff

    String windowName = "The Guitar"; //Name of the window

    namedWindow(windowName); // Create a window

    imshow(windowName, image); // Show our image inside the created window.

    waitKey(0); // Wait for any keystroke in the window

    destroyWindow(windowName); //destroy the created window

    return 0;
}