#include "common.h"
#include "memory.h"

// BoB robotics includes
#include "common/stopwatch.h"
#include "navigation/image_database.h"

// Third-party includes
#define WITH_OPENCV
#include "third_party/matplotlibcpp.h"

// Standard C++ includes
#include <sstream>

using namespace BoBRobotics;
using namespace units::time;
namespace plt = matplotlibcpp;

int bobMain(int argc, char **argv)
{
    Config config;
    config.parseArgs(argc, argv);

    const Navigation::ImageDatabase database{ config.getOutputPath() };
    auto camera = getPanoramicCamera(config);
    auto imageInput = createImageInput(config, *camera);
    auto memory = createMemory(config, imageInput->getOutputSize());
    memory->trainRoute(config.getOutputPath(), *imageInput, config.getSkipFrames());

    cv::Mat fr, processed;
    ImgProc::Mask mask;
    std::vector<double> x, y;
    Stopwatch timer, frameTimer;
    const auto *pm = dynamic_cast<PerfectMemory *>(memory.get());

    plt::figure(1);
    timer.start();
    frameTimer.start();
    while (plt::fignum_exists(1)) {
        camera->readGreyscaleFrameSync(fr);
        std::tie(processed, mask) = imageInput->preprocess(fr);
        memory->test(processed, mask);

        const auto time = static_cast<second_t>(timer.elapsed()).value();
        x.push_back(time);
        y.push_back(memory->getLowestDifference());

        plt::clf();
        plt::subplot(2, 2, 1);
        plt::imshow(processed, {{"cmap", "gray"}});
        plt::xticks(std::vector<double>{});
        plt::yticks(std::vector<double>{});
        plt::title("Current view");

        plt::subplot(2, 2, 2);
        if (pm) {
            plt::imshow(pm->getBestSnapshot(), { { "cmap", "gray" } });
            plt::xticks(std::vector<double>{});
            plt::yticks(std::vector<double>{});

            std::stringstream ss;
            ss << "Best matching snapshot (i=" << pm->getBestSnapshotIndex() << ")";
            plt::title(ss.str());
        }

        plt::subplot(2, 1, 2);
        plt::plot(x, y);
        if (time > 30) {
            plt::xlim(time - 20, time + 10);
        } else {
            plt::xlim(0, 30);
        }
        plt::xlabel("Time (s)");
        plt::ylabel("Image difference");

        std::stringstream ss;
        const auto fps = 1.0 / static_cast<second_t>(frameTimer.lap());
        ss << "FPS: " << std::setprecision(2) << std::fixed << fps.value();
        plt::title(ss.str());

        plt::pause(0.01);
    }

    return EXIT_SUCCESS;
}
