// BoB robotics includes
#include "common/logging.h"
#include "common/macros.h"
#include "video/opencvinput.h"

// Third-party includes
#include "third_party/path.h"

// Standard C++ includes
#include <iostream>
#include <string>
#include <vector>

using namespace BoBRobotics;

int
main(int argc, char **argv)
{
    if (argc < 2 || argc > 4) {
        std::cout << filesystem::path{ argv[0] } << " [video file] [size mult] [frame skip]" << std::endl;
        return EXIT_FAILURE;
    }
    BOB_ASSERT(filesystem::path{ argv[1] }.is_file());
    const double mult = (argc < 3) ? 1.0 : std::stod(argv[2]);
    BOB_ASSERT(mult > 0.0);
    const int frameSkip = (argc < 4) ? 1 : std::stoi(argv[3]) + 1;
    BOB_ASSERT(frameSkip > 0);

    Video::OpenCVInput vid{ argv[1] };
    std::vector<cv::Mat> frames;
    try {
        while (true) {
            cv::Mat fr;
            for (int i = 0; i < frameSkip; i++) {
                vid.readFrame(fr);
            }
            cv::resize(fr, fr, {}, mult, mult);
            cv::imshow("frame", fr);
            cv::waitKey(1);
            frames.emplace_back(std::move(fr));
        }
    } catch (std::runtime_error &) {
        // EOF reached
    }
    LOGI << frames.size() << " frames read";

    auto stitcher = cv::Stitcher::create(cv::Stitcher::Mode::SCANS);
    cv::Mat pano;
    const auto status = stitcher->stitch(frames, pano);
    if (status == cv::Stitcher::Status::OK) {
        cv::imwrite("pano.png", pano);
    } else {
        LOGF << "Error #" << status << " occurred";
    }
}
