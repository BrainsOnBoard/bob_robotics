// BoB robotics includes
#include "common/assert.h"
#include "navigation/image_database.h"
#include "os/keycodes.h"

// Third-party includes
#include "third_party/path.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <vector>

using namespace BoBRobotics;

int
main(int argc, char **argv)
{
    // Check we have one command-line argument
    BOB_ASSERT(argc == 2);

    // Load database metadata
    Navigation::ImageDatabase database(argv[1]);
    BOB_ASSERT(!database.empty());

    // Load images
    const auto images = database.getImages();

    // Iterate through images with arrow keys
    for (auto iter = images.begin();;) {
        cv::imshow(argv[1], *iter);
        do {
            switch (cv::waitKeyEx(50) & OS::KeyMask) {
            case OS::KeyCodes::Escape:
                return 0;
            case OS::KeyCodes::Right:
                ++iter;
                break;
            case OS::KeyCodes::Left:
                if (iter > images.begin()) {
                    --iter;
                    break;
                }
                // fall through
            default:
                continue;
            }
        } while (false);
    }
}
