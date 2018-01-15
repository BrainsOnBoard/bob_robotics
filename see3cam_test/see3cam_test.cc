// ECon_SampleApp_See3Cam_CU40.cpp : Defines the entry point for the console application.
// This sample is built using Modified VideoIO library so that the Opencv can support Y16 - Input format

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Common includes
#include "../common/timer.h"
#include "../common/v4l_camera.h"

/*bool ConvertRGIR2RGGB(Mat mInBayerRGIR,Mat &mInBayerRGGB,Mat &mIR)
{
        //Use Nearest Neibour Interpolation
        mInBayerRGGB= mInBayerRGIR.clone();
        Size szImage=mInBayerRGIR.size();
        szImage/=2;
        mIR= Mat(szImage,CV_8UC1);

        for (int Row = 0; Row < mInBayerRGIR.rows; Row+=2)
        {
                for (int Col = 0; Col < mInBayerRGIR.cols; Col+=2)
                {
                        mInBayerRGGB.at<uchar>(Row+1,Col)=mInBayerRGIR.at<uchar>(Row,Col+1);//Set the IR Data with Nearby Green
                        mIR.at<uchar>(Row/2,Col/2)=mInBayerRGIR.at<uchar>(Row+1,Col);//Set the IR Data
                }
        }
	return true;
}*/


int main()
{
    const std::string device = "/dev/video" + std::to_string(1);
    const unsigned int width = 672;
    const unsigned int height = 380;

    // Create motor
    cv::namedWindow("Camera", CV_WINDOW_NORMAL);
    cv::resizeWindow("Camera", width, height);

    See3CAM_CU40 cam(device, See3CAM_CU40::Resolution::_672x380);

    cv::Mat test2(380, 672, CV_8UC1);

    {
        Timer<> timer("Total time:");

        unsigned int frame = 0;
        for(frame = 0;; frame++) {
            void *data = nullptr;
            uint32_t sizeBytes = 0;
            if(cam.capture(data, sizeBytes)) {
                cv::Mat test(380, 672, CV_16UC1, data);


                cv::convertScaleAbs(test, test2, 0.249023);
                cv::imshow("Camera", test2);

            }
            if(cv::waitKey(1) == 27) {
                break;
            }
        }

        const double msPerFrame = timer.get() / (double)frame;
        std::cout << "FPS:" << 1000.0 / msPerFrame << std::endl;
    }
    return 0;
}

