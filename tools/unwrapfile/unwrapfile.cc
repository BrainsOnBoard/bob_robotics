// BoB robotics includes
#include "imgproc/opencv_unwrap_360.h"

// Third-party includes
#include "third_party/path.h"
#include "third_party/simpleopt.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

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
    filesystem::path tempfilename = copysound ? filepath.parent_path() / ".TEMP.AVI": outfilename;

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

void usage()
{
    std::cout << "usage: unwrapfile [--no-sound] [--extract-frames[=SKIP]] [--camera CAMERA_NAME] [--unwrapped-resolution WIDTHxHEIGHT] [file(s)]" << std::endl;
}

int main(int argc, char** argv)
{
    if (argc == 1) {
        usage();
        return EXIT_FAILURE;
    }

    enum Args
    {
        NO_SOUND,
        EXTRACT_FRAMES,
        CAMERA,
        UNWRAPPED_RESOLUTION,
        HELP
    };

    CSimpleOpt::SOption clOpts[] = {
        { NO_SOUND, "--no-sound", SO_NONE },
        { EXTRACT_FRAMES, "--extract-frames", SO_OPT },
        { CAMERA, "--camera", SO_NONE },
        { UNWRAPPED_RESOLUTION, "--unwrapped-resolution", SO_REQ_SEP },
        { HELP, "-h", SO_NONE },
        { HELP, "--help", SO_NONE },
        SO_END_OF_OPTIONS
    };

    bool copysound = true;
    bool extractFrames = false;
    unsigned int frameInterval = 1;
    std::string cameraName = "pixpro_usb";
    cv::Size unwrappedResolution(1920, 590);

    // Parse args
    CSimpleOpt args{ argc, argv, clOpts };
    try {
        while (args.Next()) {
            if (args.LastError() != SO_SUCCESS) {
                throw std::invalid_argument{ "" };
            }

            switch (args.OptionId()) {
            case NO_SOUND:
                copysound = false;
                break;
            case EXTRACT_FRAMES: {
                extractFrames = true;
                const char *arg = args.OptionArg();
                if (arg) {
                    frameInterval = std::stoi(arg);
                }
                break;
            } case CAMERA:
                cameraName = args.OptionArg();
                break;
            case UNWRAPPED_RESOLUTION: {
                const std::string arg(args.OptionArg());
                const size_t pos = arg.find('x');
                if (pos == std::string::npos) {
                    throw std::invalid_argument{ "" };
                }
                unwrappedResolution.width = std::stoi(arg.substr(0, pos));
                unwrappedResolution.height = std::stoi(arg.substr(pos + 1));
                break;
            } case HELP:
                usage();
                return EXIT_SUCCESS;
            }
        }
    } catch (std::invalid_argument &) {
        std::cerr << "Error: Invalid argument\n";
        usage();
        return EXIT_FAILURE;
    }

    if (args.FileCount() == 0) {
        std::cerr << "Error: No filenames given\n";
        return EXIT_FAILURE;
    }

    // Process filename arguments
    bool anyvideo = false;
    std::vector<FileType> ftype(args.FileCount(), FileType::skip);
    for (int i = 0; i < args.FileCount(); i++) {
        filesystem::path inputFile(args.File(i));
        if (!inputFile.exists()) {
            std::cerr << "Error: File " << inputFile.str() << " does not exist" << std::endl;
            return EXIT_FAILURE;
        }

        // Get extension and convert to lower case
        std::string ext = inputFile.extension();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == "mp4") {
            anyvideo = true;
            ftype[i] = FileType::video;
        } else if (ext == "jpg" || ext == "jpeg" || ext == "jpe")
            ftype[i] = FileType::image;
        else {
            std::cerr << "Warning : Only JPEG files and MP4 videos are supported -- skipping " << argv[i] << std::endl;
            ftype[i] = FileType::skip;
        }
    }

    if (copysound && anyvideo && !filesystem::path(FFMPEG_PATH).exists()) {
        std::cerr << "Warning: ffmpeg not found, sound will not be copied for videos" << std::endl;
        copysound = false;
    }

    std::cout << "Camera: " << cameraName << std::endl;
    std::cout << "Unwrapped resolution: " << unwrappedResolution << std::endl;

    // Process arguments
    for (int i = 0; i < args.FileCount(); i++) {
        if (ftype[i] == FileType::image) {
            unwrapJPEG(args.File(i), unwrappedResolution, cameraName);
        }
        else if (ftype[i] == FileType::video) {
            if(extractFrames) {
                unwrapMP4Frames(args.File(i), frameInterval, unwrappedResolution, cameraName);
            }
            else {
                unwrapMP4(args.File(i), copysound, unwrappedResolution, cameraName);
            }
        }
    }

    return 0;
}
