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
#include <sstream>


//! struct with the database names for convenience
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

//! unwrapping video from panoramic inpout to MP4
class VideoUnwrapper {

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

        int imgWidth = unwrappedResolution.width;
        int imgHeight = unwrappedResolution.height;
        std::stringstream unwrapped_filename;
        unwrapped_filename <<  "unwrapped_[" << imgWidth << "," << imgHeight << "]_" << filepath.filename();

        // final filename for unwrapped video
        filesystem::path outfilename = filepath.parent_path() / (unwrapped_filename.str());

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

//!reading a video file to a vector
class VideoReader {
    public:

    cv::Size m_unwrapRes;       // unwrapped resolution
    unsigned int m_outputScale; // output resolution scale
    cv::Size m_cameraRes;       // original resolution
    VideoUnwrapper uw; // unwrapper for the panoramic camera


    // default constractor - init some variables here
    VideoReader() {
        m_unwrapRes = cv::Size(90, 25);
        m_outputScale = 10;
        m_cameraRes = cv::Size(1440,1440);
    }

    //! read video to images to a vector
    std::vector<cv::Mat> readImages(int dataset_num, cv::Size unwrapRes = cv::Size(180,60)) {
        dataset_paths paths;

        // original video path
        std::string video_path_original = paths.root_path + paths.dataset_path_array[dataset_num] +  "/" + paths.dataset_path_array[dataset_num] + ".mp4";


        int imgWidth = unwrapRes.width;
        int imgHeight = unwrapRes.height;
        std::stringstream unwrapped_filename;
        unwrapped_filename <<  "unwrapped_[" << imgWidth << "," << imgHeight << "]_";

         // check if we have the file already
        std::string video_path = paths.root_path + paths.dataset_path_array[dataset_num] + "/" + unwrapped_filename.str() + paths.dataset_path_array[dataset_num] + ".mp4";
        cv::VideoCapture cap(video_path);

        // if there is no unwrapped video with the resolution, create it
        if( !cap.isOpened() ) {
            std::cout << "file does not exist, creating it..." << std::endl;
            uw.unwrapMP4(video_path_original, unwrapRes, "pixpro_usb"); // save unwrapped video with "unwrapped_" prefix

            // recreate videoreader with the now existing file
            cap =  cv::VideoCapture(video_path);
        }

        std::vector<cv::Mat> unwrapped_frames;

        if( !cap.isOpened() )
            throw "Error when reading steam_avi";

        // while the video has frames left, save frames
        for( ; ; ) {
            cv::Mat frame;
            cap >> frame;
            if(frame.empty()) {
                break;
            }
            unwrapped_frames.push_back(frame);
        }
        return unwrapped_frames;
    }
};
