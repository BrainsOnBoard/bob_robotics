#include "config.h"

// BoB robotics includes
#include "common/macros.h"

// Standard C++ includes
#include <iostream>

int bobMain(int argc, char **argv)
{
    BOB_ASSERT(argc > 1);

    for (size_t i = 1; i < (size_t) argc; i++) {
        std::cout << "Rewriting " << argv[i] << "\n";

        Config config;
        {
            cv::FileStorage fs(argv[i], cv::FileStorage::READ);
            fs["config"] >> config;
        }
        {
            cv::FileStorage fs(argv[i], cv::FileStorage::WRITE);
            fs << "config" << config;
        }
    }

    return EXIT_SUCCESS;
}
