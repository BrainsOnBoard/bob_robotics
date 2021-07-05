// BoB robotics includes
#include "navigation/image_database.h"

using namespace BoBRobotics;

int
bobMain(int argc, char **argv)
{
    // Take first command-line argument as database path
    BOB_ASSERT(argc == 2);
    const Navigation::ImageDatabase imdb{ argv[1] };
    BOB_ASSERT(!imdb.empty());

    // Iterate over entries, printing out info
    for (const auto &entry : imdb) {
        std::cout << entry.position << "\t" << entry.heading << "\t"
                  << entry.path;

        // ./write_example adds this extra field, so print it if present
        if (entry.hasExtraField("Sensor value")) {
            std::cout << "\t" << entry.getExtraField("Sensor value");
        }

        std::cout << "\n";
    }

    // Show the first image of the database
    const auto image = imdb[0].load();
    cv::imshow("Image", image);
    cv::waitKey(0);

    return EXIT_SUCCESS;
}
