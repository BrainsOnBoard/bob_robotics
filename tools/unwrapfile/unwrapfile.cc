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
void unwrapJPEG(const filesystem::path filepath, const cv::Size &unwrappedResolution, const std::string &cameraName)
{
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
void unwrapMP4(const filesystem::path filepath, bool copysound, const cv::Size &unwrappedResolution, const std::string &cameraName)
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
void unwrapMP4Frames(const filesystem::path filepath, unsigned int frameInterval, const cv::Size &unwrappedResolution, const std::string &cameraName)
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
    CLI::App app{ "A program to unwrap panoramic images and videos" };

    bool copysound = true;
    bool extractFrames = false;
    unsigned int frameInterval = 1;
    std::string cameraName = "pixpro_usb";
    std::vector<unsigned int> resolutionVec = { 1920, 590 };

    // Command-line options
    app.add_flag(
            "--no-sound",
            [&copysound](size_t count) {
                copysound = !count;
            },
            "Don't copy audio track");
    app.add_option("--extract-frames", frameInterval, "Output videos as a series of JPEGs");
    app.add_option("--camera", cameraName, "Panoramic camera's name");
    auto opt = app.add_option("--unwrapped-resolution",
                              resolutionVec,
                              "Resolution of unwrapped images/videos");
    opt->expected(2); // Width and height
    app.allow_extras(); // Accept arguments after flags

    CLI11_PARSE(app, argc, argv);
    const cv::Size unwrappedResolution{ static_cast<int>(resolutionVec[0]),
                                        static_cast<int>(resolutionVec[1]) };

    // Process filename arguments
    bool anyvideo = false;
    struct File {
        FileType type;
        filesystem::path path;

        File(const std::string &_path)
          : path(_path)
        {}
    };
    std::vector<File> files;
    files.reserve(app.remaining_size());
    for (auto &filePath : app.remaining()) {
        // Add entry to vector
        files.emplace_back(filePath);
        auto &file = files.back();

        if (!file.path.exists()) {
            std::cerr << "Error: File " << file.path.str() << " does not exist" << std::endl;
            return EXIT_FAILURE;
        }

        // Get extension and convert to lower case
        std::string ext = file.path.extension();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == "mp4") {
            anyvideo = true;
            file.type = FileType::video;
        } else if (ext == "jpg" || ext == "jpeg" || ext == "jpe")
            file.type = FileType::image;
        else {
            std::cerr << "Warning : Only JPEG files and MP4 videos are supported -- skipping "
                      << filePath << std::endl;
            file.type = FileType::skip;
        }
    }

    if (copysound && anyvideo && !filesystem::path(FFMPEG_PATH).exists()) {
        std::cerr << "Warning: ffmpeg not found, sound will not be copied for videos" << std::endl;
        copysound = false;
    }

    std::cout << "Camera: " << cameraName << std::endl;
    std::cout << "Unwrapped resolution: " << unwrappedResolution << std::endl;

    // Process arguments
    for (auto &file : files) {
        if (file.type == FileType::image) {
            unwrapJPEG(file.path, unwrappedResolution, cameraName);
        }
        else if (file.type == FileType::video) {
            if(extractFrames) {
                unwrapMP4Frames(file.path, frameInterval, unwrappedResolution, cameraName);
            }
            else {
                unwrapMP4(file.path, copysound, unwrappedResolution, cameraName);
            }
        }
    }

    return 0;
}
