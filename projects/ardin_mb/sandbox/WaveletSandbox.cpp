
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <unistd.h>
#include "third_party/wavelet2s/wavelet2s.h"

using namespace cv;
using namespace std;

// This function writes the max value of a 2d array into the max variable
void * maxval(vector<vector<double>> &arr, double &max)
{
    max = 0;
    for (unsigned int i = 0; i < arr.size(); i++) {
        for (unsigned int j = 0; j < arr[0].size(); j++) {
            if (max <= arr[i][j]) {
                max = arr[i][j];
            }
        }
    }
    return 0;
}
// This function wrotes the mac value of a 1d array into the mac variable
void * maxval1(vector<double> &arr, double &max)
{
    max = 0;
    for (unsigned int i = 0; i < arr.size(); i++) {
        if (max <= arr[i]) {
            max = arr[i];
        }
    }
    return 0;
}

// This function converts a Mat Image into a 2d vector array
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

    int rows = image.rows;
    int cols = image.cols;
    
    // Specify Wavelet transform 
    string nm = "db2";
    int level = 1;
    vector<int> length;
    vector<double> coeffs, flag;
    vector<vector<double>> vectorImage = Image2Array(image);
    // returns 1D vector that stores the output in the following format A(J) Dh(J) Dv(J) Dd(J) ..... Dh(1) Dv(1) Dd(1)
    WAVELET2S_H::dwt_2d(vectorImage, level, nm,coeffs, flag, length);

    cout << "Coeffs at 000: " <<  << "\n";

    double max;
    vector<int> length2;
    // This algorithm computes DWT of image of any given size. Together with convolution and
    // subsampling operations it is clear that subsampled images are of different length than
    // dyadic length images. In order to compute the "effective" size of DWT we do additional
    // calculations.


    cout << "length:\n";
     for( int a = 0; a < length.size(); a = a + 1 ) {
      cout << length[a] << endl;
   }

    // calculates the length of the coefficient vectors 
    WAVELET2S_H::dwt_output_dim2(length, length2, level);

    cout << "length2:\n";
     for( int a = 0; a < length.size(); a = a + 1 ) {
      cout << length2[a] << endl;
   }

    std::cout << "Size:" << coeffs.size()<< "\n";
     
     int siz = length2.size();
     int rows_n = length2[siz - 2];
     int cols_n = length2[siz - 1];
     vector<vector<double>> dwtdisp(rows_n, vector<double>(cols_n));
     dispDWT(coeffs, dwtdisp, length, length2, level);
     
//      vector<vector<double>> dwt_output = dwtdisp;
//     maxval(dwt_output, max); // max value is needed to take care of overflow which happens because
//     // of convolution operations performed on unsigned 8 bit images
//     //Displaying Scaled Image
//     // Creating Image in OPENCV
//     IplImage *cvImg; // image used for output
//     CvSize imgSize;  // size of output image
//     imgSize.width = cols_n;
//     imgSize.height = rows_n;
//     cvImg = cvCreateImage(imgSize, 8, 1);
//     // dwt_hold is created to hold the dwt output as further operations need to be
//     // carried out on dwt_output in order to display scaled images.
//     vector<vector<double>> dwt_hold(rows_n, vector<double>(cols_n));
//     dwt_hold = dwt_output;
//     // Setting coefficients of created image to the scaled DWT output values
//     for (int i = 0; i < imgSize.height; i++) {
//         for (int j = 0; j < imgSize.width; j++) {
//             if (dwt_output[i][j] <= 0.0) {
//                 dwt_output[i][j] = 0.0;
//             }
//             if (i <= (length2[0]) && j <= (length2[1])) {
//                 ((uchar *) (cvImg->imageData + cvImg->widthStep * i))[j] =
//                         (char) ((dwt_output[i][j] / max) * 255.0);
//             } else {
//                 ((uchar *) (cvImg->imageData + cvImg->widthStep * i))[j] =
//                         (char) (dwt_output[i][j]);
//             }
//         }
//     }
//     cvNamedWindow("DWT Image", 1);   // creation of a visualisation window
//     cvShowImage("DWT Image", cvImg); // image visualisation
//     cvWaitKey();
//     cvDestroyWindow("DWT Image");
//     cvSaveImage("dwt.bmp", cvImg);
//     // Finding IDWT
//     vector<vector<double>> idwt_output(rows, vector<double>(cols));
//     idwt_2d_sym(coeffs, flag, nm, idwt_output, length);
//     //Displaying Reconstructed Image
//     IplImage *dvImg;
//     CvSize dvSize; // size of output image
//     dvSize.width = idwt_output[0].size();
//     dvSize.height = idwt_output.size();
//     cout << idwt_output.size() << idwt_output[0].size() << endl;
//     dvImg = cvCreateImage(dvSize, 8, 1);
//     for (int i = 0; i < dvSize.height; i++)
//         for (int j = 0; j < dvSize.width; j++)
//             ((uchar *) (dvImg->imageData + dvImg->widthStep * i))[j] =
//                     (char) (idwt_output[i][j]);
//     cvNamedWindow("Reconstructed Image", 1);   // creation of a visualisation window
//     cvShowImage("Reconstructed Image", dvImg); // image visualisation
//     cvWaitKey();
//     cvDestroyWindow("Reconstructed Image");


//     // Display stuff

//     // String windowName = "The Guitar"; //Name of the window

//     // namedWindow(windowName); // Create a window

//     // imshow(windowName, image); // Show our image inside the created window.

//     // waitKey(0); // Wait for any keystroke in the window

//     // destroyWindow(windowName); //destroy the created window

     return 0;
}