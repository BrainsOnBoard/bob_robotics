#pragma once

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


struct dataset_paths {
    std::vector<std::string> dataset_path_array;
    std::string root_path = "../../../../rc_car_image_databases/rc_car_big/";
    dataset_paths() {

        std::string dataset0  = "20210303_150314";
        std::string dataset1  = "20210303_153749";
        std::string dataset2  = "20210308_122125";
        std::string dataset3  = "20210308_124836";
        std::string dataset4  = "20210322_155544";
        std::string dataset5  = "20210322_170743";
        std::string dataset6  = "20210414_151116";
        std::string dataset7  = "20210420_135721";
        std::string dataset8  = "20210420_141940";
        std::string dataset9  = "20210422_134815";
        std::string dataset10 = "20210426_162152";
        std::string dataset11 = "20210426_164219";
        std::string dataset12 = "20210511_151514";
        std::string dataset13 = "20210511_153933";
        std::string dataset14 = "20210525_141844";

        dataset_path_array.push_back(dataset0);
        dataset_path_array.push_back(dataset1);
        dataset_path_array.push_back(dataset2);
        dataset_path_array.push_back(dataset3);
        dataset_path_array.push_back(dataset4);
        dataset_path_array.push_back(dataset5);
        dataset_path_array.push_back(dataset6);
        dataset_path_array.push_back(dataset7);
        dataset_path_array.push_back(dataset8);
        dataset_path_array.push_back(dataset9);
        dataset_path_array.push_back(dataset10);
        dataset_path_array.push_back(dataset11);
        dataset_path_array.push_back(dataset12);
        dataset_path_array.push_back(dataset13);
        dataset_path_array.push_back(dataset14);
    }
};

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

// reading a video file
class VideoReader {
    public:

    cv::Size unwrapRes;
    unsigned int outputScale;
    VideoUnwrapper uw;

    VideoReader() {
        unwrapRes = cv::Size(90, 25);
        outputScale = 10;
    }

    // read video to images
    std::vector<cv::Mat> readImages(int dataset_num, bool unwrap,bool createfile, cv::Size unwrapRes = cv::Size(90,25)) {
        dataset_paths paths;
        std::string video_path = paths.root_path + paths.dataset_path_array[dataset_num] +  "/" + paths.dataset_path_array[dataset_num] + ".mp4";

        if (!unwrap) {

            if (createfile) {
                uw.unwrapMP4(video_path, unwrapRes, "pixpro_usb");
                video_path = paths.root_path + paths.dataset_path_array[dataset_num] +  "/unwrapped_" + paths.dataset_path_array[dataset_num] + ".mp4";
            } else {
                video_path = paths.root_path + paths.dataset_path_array[dataset_num] +  "/unwrapped_" + paths.dataset_path_array[dataset_num] + ".mp4";
            }
        }

        cv::VideoCapture cap(video_path);
        std::vector<cv::Mat> unwrapped_frames;

        if( !cap.isOpened() )
            throw "Error when reading steam_avi";

        const cv::Size cameraRes(1440,1440);
        const unsigned int outputScale = 10;

        // Create panoramic camera and suitable unwrapper
        BoBRobotics::ImgProc::OpenCVUnwrap360 unwrapper(cameraRes,unwrapRes, "pixpro_usb");

         // Create images
        cv::Mat originalImage(cv::Size(1440,1440), CV_8UC3);

        for( ; ; ) {
            cv::Mat frame;
            cap >> frame;
            if(frame.empty())
                break;

            if (unwrap) {
                cv::Mat outputImage(unwrapRes, CV_8UC3);
                unwrapper.unwrap(frame, outputImage);
                unwrapped_frames.push_back(outputImage);

            } else {

                unwrapped_frames.push_back(frame);
            }

        }

        return unwrapped_frames;
    }
};
