
// BoB robotics includes
#include "common/assert.h"
#include "common/main.h"

// Third-party includes
#include "third_party/matplotlibcpp.h"

// Vicon Datastream SDK includes
#include "DataStreamClient.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cmath>
#include <ctime>

// Standard C++ includes
#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace ViconDataStreamSDK::CPP;
using namespace std::literals;

namespace plt = matplotlibcpp;

int
bob_main(int argc, char **argv)
{
    const std::string ip = (argc == 1) ? "10.0.0.2" : argv[1];

    // Make a new client
    Client client;
    const std::string host = ip + ":801";

    // Connect to a server
    std::cout << "Connecting to " << host << " ..." << std::endl;
    do {
        if (client.Connect(host).Result == Result::Success) {
            std::cout << "Connected!" << std::endl;
            break;
        }

        std::cout << "Warning - connect failed..." << std::endl;
        std::cout << ".";
        std::this_thread::sleep_for(500ms);
    } while (!client.IsConnected().Connected);

    // Discover the version number
    const auto version = client.GetVersion();
    std::cout << "Version: " << version.Major << "."
              << version.Minor << "."
              << version.Point << std::endl;

    // Get data for labelled and unlabelled markers
    client.EnableMarkerData();
    client.EnableUnlabeledMarkerData();

    // Keep adding objects until stopped
    std::vector<std::pair<std::vector<double>, std::vector<double>>> objects;
    while (true) {
        std::cout << "Waiting for frame..." << std::endl;
        while (client.GetFrame().Result != Result::Success) {
            std::this_thread::sleep_for(500ms);
        }

        /*
             * There shouldn't be any labelled markers (i.e. Vicon tracking
             * objects) in the arena.
             */
        unsigned int numLabeledMarkers = client.GetLabeledMarkerCount().MarkerCount;
        BOB_ASSERT(numLabeledMarkers == 0);

        // Get the unlabeled markers
        unsigned int numUnlabeledMarkers = client.GetUnlabeledMarkerCount().MarkerCount;
        std::vector<double> markerX, markerY;
        markerX.reserve(numUnlabeledMarkers);
        markerY.reserve(numUnlabeledMarkers);
        std::cout << "  Unlabeled Markers (" << numUnlabeledMarkers << "):" << std::endl;
        for (unsigned int i = 0; i < numUnlabeledMarkers; ++i) {
            const auto ret = client.GetUnlabeledMarkerGlobalTranslation(i);
            markerX.push_back(ret.Translation[0]);
            markerY.push_back(ret.Translation[1]);

            std::cout << "    Marker #" << i << ": ("
                      << ret.Translation[0] << ", "
                      << ret.Translation[1] << ", "
                      << ret.Translation[2] << ")" << std::endl;
        }

        // We need at least three to make a shape
        BOB_ASSERT(numUnlabeledMarkers >= 3);

        // Ask user how many markers to use
        size_t numVertices{};
        do {
            std::cout << std::endl << "How many markers do you wish to select ["
                      << numUnlabeledMarkers << "]: " << std::flush;
            std::string input;
            std::getline(std::cin, input);
            if (input.empty()) {
                // Default to using all markers
                numVertices = numUnlabeledMarkers;
            } else {
                try {
                    numVertices = std::stoul(input);
                } catch (std::invalid_argument &) {
                    // Not a number
                    continue;
                }
                if (numVertices < 3 || numVertices > numUnlabeledMarkers) {
                    continue;
                }
            }
        } while (false);

        // Get user to order the vertices by clicking on them
        plt::plot(markerX, markerY, "b+");
        plt::title("Click each of the vertices in turn");
        for (size_t i = 0; i < numVertices; i++) {
            // Get position of mouse click
            const auto positions = plt::ginput();
            if (!plt::fignum_exists(1)) {
                return EXIT_SUCCESS;
            }

            // Find nearest marker to click position
            size_t minIndex;
            double minDistance = std::numeric_limits<double>::infinity();
            for (size_t j = i; j < numUnlabeledMarkers; j++) {
                const auto dist = ::hypot(markerX[j] - positions[0][0], markerY[j] - positions[0][1]);
                if (dist < minDistance) {
                    minIndex = j;
                    minDistance = dist;
                }
            }

            // Reorder vectors appropriately
            if (minIndex != i) {
                std::swap(markerX[i], markerX[minIndex]);
                std::swap(markerY[i], markerY[minIndex]);
            }
        }
        plt::close();

        // Draw outline of the object
        plt::figure();
        std::vector<double> vertexX, vertexY;
        std::copy_n(markerX.cbegin(), numVertices, std::back_inserter(vertexX));
        std::copy_n(markerY.cbegin(), numVertices, std::back_inserter(vertexY));
        vertexX.push_back(vertexX[0]);
        vertexY.push_back(vertexY[0]);
        plt::plot(vertexX, vertexY, "g");
        plt::title("Close when you're ready");
        plt::show();

        // Drop the repeated last elements from these vectors for saving
        vertexX.resize(numVertices);
        vertexY.resize(numVertices);

        // User input to decide what to do next
        do {
            std::cout << "Would you like to enter another object? [y|n|r = redo|q = quit] " << std::flush;
            char c;
            std::cin >> c;
            switch (c) {
            case 'n': // Save and quit
                objects.emplace_back(std::move(vertexX), std::move(vertexY));

                {
                    const std::string filename = "objects.yaml";
                    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

                    // Get current date and time
                    std::time_t now = std::time(nullptr);
                    char timeStr[sizeof("0000-00-00 00:00:00")];
                    BOB_ASSERT(0 != std::strftime(timeStr, sizeof(timeStr), "%F %T", std::localtime(&now)));

                    fs << "metadata"
                       << "{"
                       << "time" << timeStr
                       << "}";
                    fs << "objects"
                       << "[";
                    for (auto &o : objects) {
                        fs << "[";
                        for (size_t i = 0; i < o.first.size(); i++) {
                            fs << "[:" << o.first[i] << o.second[i] << "]";
                        }
                        fs << "]";
                    }
                    fs << "]";

                    std::cout << "Saving to " << filename << std::endl;
                }

                return EXIT_SUCCESS;
            case 'y': // Add another object
                objects.emplace_back(std::move(markerX), std::move(markerY));
                // Fall through
            case 'r': // Redo object
                break;
            case 'q': // Quit without saving
                std::cout << "Exiting without saving" << std::endl;
                return EXIT_SUCCESS;
            default:
                continue;
            }
        } while (false);
    }
}
