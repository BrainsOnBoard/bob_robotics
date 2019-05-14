// BoB robotics includes
#include "common/macros.h"
#include "navigation/image_database.h"
#include "os/keycodes.h"

// Third-party includes
#include "third_party/path.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <sstream>
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
    std::cout << "Loading images..." << std::endl;
    const auto images = database.getImages();

    // Iterate through images with arrow keys
    cv::namedWindow(argv[0]);
    for (size_t i = 0; i < database.size();) {
        std::stringstream ss;
        const auto pos = database[i].position;
        ss << database.getName() << " [" << 1 + i << "/" << database.size() << "]"
           << " (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")";
        cv::imshow(argv[0], images[i]);
        cv::setWindowTitle(argv[0], ss.str());
        do {
            switch (cv::waitKeyEx(50) & OS::KeyMask) {
            case OS::KeyCodes::Escape:
                return 0;
            case OS::KeyCodes::Right:
                ++i;
                break;
            case OS::KeyCodes::Left:
                if (i > 0) {
                    --i;
                    break;
                }
                // fall through
            default:
                continue;
            }
        } while (false);
    }
}
