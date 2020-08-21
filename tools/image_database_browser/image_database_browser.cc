// BoB robotics includes
#include "common/macros.h"
#include "navigation/image_database.h"
#include "os/keycodes.h"

// Third-party includes
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
    // Check we have one command-line argument
    BOB_ASSERT(argc == 2);

    // Load database metadata
    Navigation::ImageDatabase database(argv[1]);
    BOB_ASSERT(!database.empty());

    // Load images
    std::cout << "Loading images..." << std::endl;
    const auto images = database.getImages();

    // Iterate through images with arrow keys
    cv::namedWindow(argv[0]);
    for (size_t i = 0; i < database.size();) {
        std::stringstream ssTitle, ssNumber;
        const auto pos = database[i].position;
        ssTitle << database.getName() << " [" << i << "/" << database.size() << "]"
           << " (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")";
        cv::imshow(argv[0], images[i]);
        cv::setWindowTitle(argv[0], ssTitle.str());

        size_t newi = i;
        do {
            const auto key = cv::waitKeyEx(50) & OS::KeyMask;
            if (key >= '0' && key <= '9') {
                ssNumber << static_cast<char>(key);
                std::cout << "Frame: " << ssNumber.str() << std::endl;
                continue;
            }

            switch (key) {
            case OS::KeyCodes::Escape:
                return EXIT_SUCCESS;
            case OS::KeyCodes::Right:
                if (newi < database.size() - 1) {
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
                    newi = std::min(std::stoul(ssNumber.str()), database.size() - 1);
                }
                break;
            default:
                continue;
            }

            ssNumber.str("");
            ssNumber.clear();
        } while (i == newi);
        i = newi;
    }
    return EXIT_SUCCESS;
}
