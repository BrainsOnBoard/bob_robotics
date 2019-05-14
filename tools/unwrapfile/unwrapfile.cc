// BoB robotics includes
#include "imgproc/opencv_unwrap_360.h"

// Third-party includes
#include "third_party/path.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cassert>
#include <cstdlib>

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

/*
 * ffmpeg is used for copying the audio stream from the original to the
 * unwrapped video
 */
#ifndef FFMPEG_PATH
#define FFMPEG_PATH "/usr/bin/ffmpeg"
#endif

// Anonymous namespace
namespace
{
enum class FileType {
    skip,
    image,
    video
};

void saveMaxQualityJPG(const std::string &filename, const cv::Mat &image)
{
    static std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 100};
    cv::imwrite(filename, image, params);
}

/* unwrap a JPEG file */
void unwrapJPEG(const char *filepathRaw, const cv::Size &unwrappedResolution, const std::string &cameraName)
{
    const filesystem::path filepath(filepathRaw);

    // read image into memory
    cv::Mat im = cv::imread(filepath.str(), cv::IMREAD_COLOR);

    // matrix to store unwrapped image
    cv::Mat imunwrap(unwrappedResolution, im.type());

    // Create unwrapper
    BoBRobotics::ImgProc::OpenCVUnwrap360 unwrapper(im.size(), unwrappedResolution, cameraName);

    // Unwrap image
    unwrapper.unwrap(im, imunwrap);

    // Save file
    filesystem::path outfilename = filepath.parent_path() / ("unwrapped_" + filepath.filename());
    std::cout << "Saving image to " << outfilename.str() << "..." << std::endl;
    saveMaxQualityJPG(outfilename.str(), imunwrap);
}

/* unwrap an MP4 video */
void unwrapMP4(const char *filepathRaw, bool copysound, const cv::Size &unwrappedResolution, const std::string &cameraName)
{
    filesystem::path filepath(filepathRaw);

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
    filesystem::path tempfilename = copysound ? filepath.parent_path() / ".TEMP.MP4": outfilename;

    // start writing to file
    std::cout << "Saving video to " << outfilename << "..." << std::endl;
    cv::VideoWriter writer(tempfilename.str(), 0x21, cap.get(cv::CAP_PROP_FPS), unwrappedResolution);
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

    if (copysound) {
        // attempt to copy audio stream from original to new file with ffmpeg
        int stat = system((FFMPEG_PATH " -y -i \"" + tempfilename.str() + "\" -i \"" +
                           filepath.str() +
                           "\" -map 0:v -map 1:a -c copy -shortest \"" +
                           outfilename.str() + "\" >/dev/null 2>&1")
                                  .c_str());
        if (stat != 0) {
            std::cerr << "Error (" << stat
                 << ") occurred while copying audio with ffmpeg" << std::endl;

            /*
             * just rename the temporary file to the output file, so there won't
             * be audio but at least there'll be video
             */
            rename(tempfilename.str().c_str(), outfilename.str().c_str());
        } else {
            // successfully copied audio, so delete temporary file
            remove(tempfilename.str().c_str());
        }
    }
}

/* unwrap an MP4 video, saving frames to JPG */
void unwrapMP4Frames(const char *filepathRaw, unsigned int frameInterval, const cv::Size &unwrappedResolution, const std::string &cameraName)
{
    const filesystem::path filepath(filepathRaw);

    // open video file
    cv::VideoCapture cap(filepath.str());

    // read in first frame
    cv::Mat fr;
    cap >> fr;

    // matrix to store unwrapped image
    cv::Mat imunwrap(unwrappedResolution, fr.type());

    // Create unwrapper
    BoBRobotics::ImgProc::OpenCVUnwrap360 unwrapper(fr.size(), unwrappedResolution, cameraName);

    // Extract title from filename
    const size_t titlePos = filepath.filename().find_last_of(".");
    const std::string filetitle = filepath.filename().substr(0, titlePos);


    // Loop through frames
    unsigned int i = 0;
    for(unsigned int f = 0; !fr.empty(); f++) {
        if((f % frameInterval) == 0) {
            // Unwrap frame
            unwrapper.unwrap(fr, imunwrap);

            // Build frame index string
            char imageIndexString[10];
            sprintf(imageIndexString, "%05u", i++);

            // Save frame as jpg
            const filesystem::path outfilename = filepath.parent_path() / ("unwrapped_" + filetitle + "_" + imageIndexString + ".jpg");
            saveMaxQualityJPG(outfilename.str(), imunwrap);
        }

        // read next frame
        cap >> fr;
    }

    // dispose of writer and reader when finished
    cap.release();
}
}   // Anonymous namespace

int main(int argc, char** argv)
{
    if (argc == 1) {
        std::cout << "usage: unwrapfile [--no-sound] [--extract-frames SKIP] [--camera CAMERA_NAME] [--unwrapped-resolution WIDTH HEIGHT] [file(s)]" << std::endl;
        return 0;
    }

    std::vector<FileType> ftype(argc - 1, FileType::skip);
    bool anyvideo = false;
    bool copysound = true;
    bool extractFrames = true;
    unsigned int frameInterval = 1;
    std::string cameraName = "pixpro_usb";
    cv::Size unwrappedResolution(1920, 590);
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--no-sound") == 0) {
            copysound = false;
            continue;
        }

        if(strcmp(argv[i], "--camera") == 0) {
            // Check there's another parameter to read
            assert(i < (argc - 2));

            // Read NEXT parameter as camera name and skip over it
            cameraName = argv[i + 1];
            i++;
            continue;
        }

        if(strcmp(argv[i], "--extract-frames") == 0) {
            // Check there's another parameter to read
            assert(i < (argc - 2));

            // Set extract frames flag, read NEXT parameter as frame interval and skip over it :)
            extractFrames = true;
            frameInterval = std::stoi(argv[i + 1]);
            i++;

            continue;
        }

        if(strcmp(argv[i],"--unwrapped-resolution") == 0) {
            // Check there's another 2 parameters to read
            assert(i < (argc - 3));

            // Read NEXT two parameters as width and height
            unwrappedResolution.width = std::stoi(argv[i + 1]);
            unwrappedResolution.height = std::stoi(argv[i + 2]);
            i += 2;
            continue;
        }

        filesystem::path inputFile(argv[i]);
        if (!inputFile.exists()) {
            std::cerr << "Error: File " << inputFile.str() << " does not exist" << std::endl;
            return 1;
        }

        // Get extension and convert to lower case
        std::string ext = inputFile.extension();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == "mp4") {
            anyvideo = true;
            ftype[i - 1] = FileType::video;
        } else if (ext == "jpg" || ext == "jpeg" || ext == "jpe")
            ftype[i - 1] = FileType::image;
        else {
            std::cerr << "Warning : Only JPEG files and MP4 videos are supported -- skipping " << argv[i] << std::endl;
            ftype[i - 1] = FileType::skip;
        }
    }

    if (copysound && anyvideo && !filesystem::path(FFMPEG_PATH).exists()) {
        std::cerr << "Warning: ffmpeg not found, sound will not be copied for videos" << std::endl;
        copysound = false;
    }

    std::cout << "Camera: " << cameraName << std::endl;
    std::cout << "Unwrapped resolution: " << unwrappedResolution << std::endl;
    // Process arguments
    for (int i = 0; i < argc - 1; i++) {
        if (ftype[i] == FileType::image) {
            unwrapJPEG(argv[i + 1], unwrappedResolution, cameraName);
        }
        else if (ftype[i] == FileType::video) {
            if(extractFrames) {
                unwrapMP4Frames(argv[i + 1], frameInterval, unwrappedResolution, cameraName);
            }
            else {
                unwrapMP4(argv[i + 1], copysound, unwrappedResolution, cameraName);
            }
        }
    }

    return 0;
}
