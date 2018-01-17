#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Common includes
#include "../common/timer.h"
#include "../common/v4l_camera.h"

/*#define R(x, y, w)  output.data[0 + 3 * ((x) + (w) * (y))]
#define G(x, y, w)  output.data[1 + 3 * ((x) + (w) * (y))]
#define B(x, y, w)  output.data[2 + 3 * ((x) + (w) * (y))]

#define Bay(x, y, w) dataPixels[(x) + (w) * (y)]*/

void convertRGIR2RGGB(cv::Mat &bayer)
{
    for (int row = 0; row < bayer.rows; row+=2)
    {
        for (int col = 0; col < bayer.cols; col+=2)
        {
            bayer.at<uchar>(row + 1, col) = bayer.at<uchar>(row, col + 1);
        }
    }
}

int main()
{
    const std::string device = "/dev/video" + std::to_string(1);
    const See3CAM_CU40::Resolution res = See3CAM_CU40::Resolution::_672x380;

    // Create window
    const unsigned int width = See3CAM_CU40::getWidth(res);
    const unsigned int height = See3CAM_CU40::getHeight(res);
    const unsigned int outputWidth = width;
    const unsigned int outputHeight = height;

    cv::namedWindow("Camera", CV_WINDOW_NORMAL);
    cv::resizeWindow("Camera", outputWidth, outputHeight);


    See3CAM_CU40 cam(device, res);

    cv::Mat rescaled(height, width, CV_8UC1);
    cv::Mat output(outputHeight, outputWidth, CV_8UC3);

    {
        Timer<> timer("Total time:");

        unsigned int frame = 0;
        for(frame = 0;; frame++) {
            // Read data and size (in bytes) from camera
            // **NOTE** these pointers are only valid within one frame
            void *data = nullptr;
            uint32_t sizeBytes = 0;
            if(cam.capture(data, sizeBytes)) {
                assert(sizeBytes == (width * height * sizeof(uint16_t)));

                // Add OpenCV header to data
                cv::Mat input(height, width, CV_16UC1, data);

                //Convert to 8 Bit: Scale the 10 Bit (1024) Pixels into 8 Bit(255) (255/1024)= 0.249023
                cv::convertScaleAbs(input, rescaled, 0.249023);

                // Overwrite the IR data with duplicated green channel to convert to standard RGGB Bayer format
                convertRGIR2RGGB(rescaled);

                //Actual Bayer format BG but Opencv uses BGR & Not RGB So taking RG Bayer format
                cv::demosaicing(rescaled, output, cv::COLOR_BayerRG2BGR);

                cv::imshow("Camera", output);
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

