// BoB robotics includes
#include "imgproc/opencv_unwrap_360.h"

// Third-party includes
#include "third_party/CLI11.hpp"
#include "third_party/path.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

class VideoUnwrapper {
    enum class FileType {
        skip,
        image,
        video
    };

    public:
    /* unwrap an MP4 video */
    void unwrapMP4(const filesystem::path &filepath, const cv::Size &unwrappedResolution, const std::string &cameraName)
    {
        // open video file
        cv::VideoCapture cap(filepath.str());

        // read in first frame
        cv::Mat fr;
        cap >> fr;

        // matrix to store unwrapped image
        cv::Mat imunwrap(unwrappedResolution, fr.type());

        // Create unwrapper
        BoBRobotics::ImgProc::OpenCVUnwrap360 unwrapper(fr.size(), unwrappedResolution, cameraName);

        // final filename for unwrapped video
        filesystem::path outfilename = filepath.parent_path() / ("unwrapped_" + filepath.filename());

        // temporary file name to which we write initially
        bool copysound = false;
        filesystem::path tempfilename = copysound ? filepath.parent_path() / (".TMP." + filepath.filename())
                                                : outfilename;

        // start writing to file
        std::cout << "Saving video to " << outfilename << "..." << std::endl;
        cv::VideoWriter writer(tempfilename.str(),
                            cv::VideoWriter::fourcc('F', 'M', 'P', '4'),
                            cap.get(cv::CAP_PROP_FPS),
                            unwrappedResolution);
        if (!writer.isOpened()) {
            std::cerr << "Error: Could not open file for writing" << std::endl;
            return;
        }

        // write first unwrapped frame
        writer.write(imunwrap);

        // unwrap successive frames and write to file
        while(!fr.empty()) {
            unwrapper.unwrap(fr, imunwrap);
            writer.write(imunwrap);

            cap >> fr;
        }

        // dispose of writer and reader when finished
        cap.release();
        writer.release();


    }
};
