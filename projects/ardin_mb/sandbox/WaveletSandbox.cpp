
#include "third_party/wavelet2s/wavelet2s.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <unistd.h>
#include <Eigen/Dense>

using namespace cv;
using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Map;
using Eigen::Matrix;

/*
HELP FUNCTION FO WAVELET EXAMPLE
This function writes the max value of a 2d array into the max variable 
*/
void *
maxval(vector<vector<double>> &arr, double &max)
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
/*
HELP FUNCTION FO WAVELET EXAMPLE
This function writes the max value of a 1d array into the max variable 
*/
void *
maxval1(vector<double> &arr, double &max)
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
vector<vector<double>>
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

Mat Array2Image(vector<vector<double>> vecImage)
{
    Mat matImage(vecImage.size(), vecImage.at(0).size(), CV_64FC1);
    for(int i=0; i<matImage.rows; ++i)
    {
        for(int j=0; j<matImage.cols; ++j)
        {
            matImage.at<double>(i, j) = vecImage.at(i).at(j);
        }
          
    }
    return matImage;
}

MatrixXd Array2Matrix(vector<vector<double>> data)
{
    MatrixXd eMatrix(data.size(), data[0].size());
    for (int i = 0; i < data.size(); ++i)
        eMatrix.row(i) = VectorXd::Map(&data[i][0], data[0].size());
    return eMatrix;
}

// This function prints x to the console
template<class T>
void
LOG(T x)
{
    cout << x << "\n";
}

// This function slices out a part of a given vector
template<class T>
vector<T>
slice(vector<T> const &v, int start, int stop)
{
    auto first = v.cbegin() + start;
    auto last = v.cbegin() + 1 + stop;

    vector<T> vec(first, last);
    return vec;
}

main(int argc, char **argv)
{
    // Read the image file
    Mat image = imread("../lena.png");

    // Check for failure
    if (image.empty()) {
        cout << "Could not open or find the image" << endl;
        cin.get(); //wait for any key press
        return -1;
    }

    // Get image dimensions
    int rows = image.rows;
    int cols = image.cols;
    // int nc = number of channels

    // Specify Wavelet transform
    string nm = "db5";
    int level = 2;
    vector<int> length;
    vector<double> coeffs, flag;
    vector<vector<double>> vectorImage = Image2Array(image);

    // returns 1D vector that stores the output in the following format A(J) Dh(J) Dv(J) Dd(J) ..... Dh(1) Dv(1) Dd(1)
    dwt_2d(vectorImage, level, nm, coeffs, flag, length);

    vector<int> length2;
    // calculates the length of the coefficient vectors
    dwt_output_dim2(length, length2, level);
    
    // setup the new image dimensions for display
    int siz = length2.size();
    int rows_n = length2[siz - 2];
    int cols_n = length2[siz - 1];

    vector<vector<double> > dwtdisp(rows_n, vector<double>(cols_n));
    dispDWT(coeffs, dwtdisp, length, length2, level);
    MatrixXd M = Array2Matrix(dwtdisp);
    
    MatrixXd sub = M.bottomLeftCorner(M.rows()-257,255);
    double maxValue = sub.maxCoeff();
    for (int i = 0; i<sub.rows(); i++)
    {
        for (int j = 0; j<sub.cols(); j++)
        {
            if (sub(i,j) <= 0)
            {
                sub(i,j) = 0;
            }
            else 
            {
                sub(i,j) = sub(i,j)/maxValue;
            }
        
        }
            
    }
    
    LOG(sub.rows());
    LOG(sub.cols());
    LOG(sub.maxCoeff());
    LOG(sub.minCoeff());
    
    //Mat img = Array2Image(Matrix2Array(sub));

    LOG(dwtdisp.at(0).at(0));
    LOG(dwtdisp.size());
    LOG(dwtdisp.at(0).size());
    LOG(M.maxCoeff());
    LOG(M.minCoeff());
    //LOG(M.min());
    vector<double> subset = slice(coeffs, 0, 360);


    
    Mat subImage;
    eigen2cv(sub,subImage);
    //std::cout << "Size:" << coeffs.size() << "\n";


    
//     // write coeffs in the right location for the image to be displayed
    

    // vector<vector<double>> dwt_output = dwtdisp;
    // maxval(dwt_output, max); // max value is needed to take care of overflow which happens because

    //Mat newImage = Array2Image(dwt_output);

    namedWindow("Display window", WINDOW_AUTOSIZE);
    imshow("Display window", subImage);

    waitKey(0);

    // return 0;
}