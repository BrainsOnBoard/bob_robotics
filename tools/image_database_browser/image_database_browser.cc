// BoB robotics includes
#include "common/macros.h"
#include "navigation/image_database.h"
#include "os/keycodes.h"

// Third-party includes
#include "third_party/CLI11.hpp"
#include "third_party/path.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <sstream>
#include <vector>

using namespace BoBRobotics;

int bobMain(int argc, char **argv)
{
    size_t frameSkip = 1;

    // Handle command-line arguments
    CLI::App app{ "Tool for examining the contents of an image database." };
    app.add_option("-s,--skip-frames", frameSkip, "Number of frames to skip when reading");
    app.allow_extras();
    CLI11_PARSE(app, argc, argv);
    BOB_ASSERT(frameSkip > 0);
    if (app.remaining_size() != 1) {
        std::cout << app.help();
        return 1;
    }

    // Load database metadata
    const Navigation::ImageDatabase database{ app.remaining()[0] };
    BOB_ASSERT(!database.empty());

    // Load images
    std::cout << "Loading images..." << std::endl;
    const auto images = database.loadImages({}, frameSkip, false);

    // Iterate through images with arrow keys
    cv::namedWindow(argv[0]);
    for (size_t i = 0; i < images.size();) {
        std::stringstream ssTitle, ssNumber;
        const auto pos = database[i * frameSkip].position;
        ssTitle << database.getName() << " [" << i << "/" << images.size() << "]"
                << " (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")";
        cv::imshow(argv[0], images[i]);
        cv::setWindowTitle(argv[0], ssTitle.str());

        size_t newi = i;
        do {
            /*
             * If the user presses a number key, we append this digit to a
             * stringstream and when the user then presses 'g' we jump to that
             * frame number. For example, if the user enters '1', '0', '0', 'g'
             * then we jump to frame number 100 (the same way you jump to a line
             * in vim).
             */
            auto key = cv::waitKeyEx(50);
            if (key == -1) {
                // No key pressed, so try again
                continue;
            }
            key &= OS::KeyMask;

            if (key >= '0' && key <= '9') {
                ssNumber << static_cast<char>(key);
                std::cout << "Frame: " << ssNumber.str() << std::endl;
                continue;
            }

            switch (key) {
            case OS::KeyCodes::Escape:
                return EXIT_SUCCESS;
            case OS::KeyCodes::Right:
                if (newi < images.size() - 1) {
                    newi++;
                }
                break;
            case OS::KeyCodes::Left:
                if (newi > 0) {
                    newi--;
                }
                break;
            case 'g':
                if (!ssNumber.str().empty()) {
                    newi = std::min(std::stoul(ssNumber.str()), images.size() - 1);
                }
                break;
            }

            ssNumber.str("");
            ssNumber.clear();
        } while (i == newi);
        i = newi;
    }
    return EXIT_SUCCESS;
}
