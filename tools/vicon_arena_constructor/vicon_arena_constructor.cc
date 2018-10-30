// Vicon Datastream SDK includes
#include "DataStreamClient.h"

// Third-party includes
#include "third_party/matplotlibcpp.h"

// Standard C++ includes
#include <chrono>
#include <exception>
#include <iostream>
#include <thread>

using namespace ViconDataStreamSDK::CPP;
using namespace std::literals;

namespace plt = matplotlibcpp;

int
main()
{
    try {
        // Make a new client
        Client MyClient;
        const std::string host = "10.0.0.7:801";

        // Connect to a server
        std::cout << "Connecting to " << host << " ..." << std::flush;
        while (!MyClient.IsConnected().Connected) {
            if (MyClient.Connect(host).Result == Result::Success) {
                break;
            }

            std::cout << "Warning - connect failed..." << std::endl;
            std::cout << ".";
            std::this_thread::sleep_for(500ms);
        }

        std::cout << "Connected!" << std::endl;
        // Discover the version number
        Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
        std::cout << "Version: " << _Output_GetVersion.Major << "."
                  << _Output_GetVersion.Minor << "."
                  << _Output_GetVersion.Point << std::endl;

        MyClient.EnableMarkerData();
        MyClient.EnableUnlabeledMarkerData();

        std::cout << "Waiting for frame..." << std::endl;
        while (MyClient.GetFrame().Result != Result::Success) {
            std::this_thread::sleep_for(500ms);
        }

        // Get the labeled markers
        unsigned int numLabeledMarkers = MyClient.GetLabeledMarkerCount().MarkerCount;
        std::cout << "    Labeled Markers (" << numLabeledMarkers << "):" << std::endl;
        for (unsigned int i = 0; i < numLabeledMarkers; ++i) {
            // Get the global marker translation
            const auto ret = MyClient.GetLabeledMarkerGlobalTranslation(i);

            std::cout << "      Marker #" << i << ": ("
                      << ret.Translation[0] << ", "
                      << ret.Translation[1] << ", "
                      << ret.Translation[2] << ")" << std::endl;
        }

        // Get the unlabeled markers
        unsigned int numUnlabeledMarkers = MyClient.GetUnlabeledMarkerCount().MarkerCount;
        std::vector<double> markerX, markerY;
        markerX.reserve(numUnlabeledMarkers);
        markerY.reserve(numUnlabeledMarkers);
        std::cout << "    Unlabeled Markers (" << numUnlabeledMarkers << "):" << std::endl;
        for (unsigned int i = 0; i < numUnlabeledMarkers; ++i) {
            const auto ret = MyClient.GetUnlabeledMarkerGlobalTranslation(i);
            markerX.push_back(ret.Translation[0]);
            markerY.push_back(ret.Translation[1]);

            std::cout << "      Marker #" << i << ": ("
                      << ret.Translation[0] << ", "
                      << ret.Translation[1] << ", "
                      << ret.Translation[2] << ")" << std::endl;
        }

        // Plot positions of unlabeled markers
        plt::plot(markerX, markerY, "b+");
        plt::show();

    } catch (std::exception &e) {
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
