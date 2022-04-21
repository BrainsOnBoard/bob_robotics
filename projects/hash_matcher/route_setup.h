#pragma once;
// includes
#include <vector>
#include <algorithm>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <bitset>
#include <string>
#include <iostream>
#include <random>
#include <chrono>


#include "video/panoramic.h"
#include "include/common/string.h"
#include "include/common/macros.h"
#include "imgproc/dct_hash.h"
#include "imgproc/roll.h"
#include "common/stopwatch.h"
#include "video_unwrapper.h"
#include "third_party/units.h"

using namespace units;
using namespace units::literals;
using namespace units::angle;
using namespace units::length;
using meter_t = units::length::meter_t;
using millimeter_t = units::length::millimeter_t;
using degree_t = units::angle::degree_t;

// database entry struct
struct Database_entry {
    float s_timestamp;
    millimeter_t s_x;
    millimeter_t s_y;
    millimeter_t s_z;
    degree_t s_heading;
    degree_t s_pitch;
    degree_t s_roll;
    float s_speed;
    int s_steering;
    std::string s_filename;
    int s_gps_quality;
    std::string s_utm_zone;
    millimeter_t s_fx;
    millimeter_t s_fy;
    millimeter_t s_fz;
    degree_t s_gps_h;
    degree_t s_corr_heading;

    Database_entry() { }
};

struct RouteNode {
    millimeter_t x;
    millimeter_t y;
    millimeter_t z;
    degree_t heading;
    std::bitset<64> image_hash;
    cv::Mat image;
    int node_number;

    //! Eauclidean distance between 2 nodes - to be done
    static int distance(RouteNode node1, RouteNode node2) {
        // euclidean distance
        return 0;
    }

    static int angle_distance(RouteNode node1, RouteNode node2) {
        // absolut angle distance between 2 poses
        return 0;
    }
};

// a csv reader to read database files
struct CSVReader {
    static std::vector<Database_entry> loadCSV(int dataset_num) {
        dataset_paths paths;
        std::vector<Database_entry> entries;
        std::string video_path = paths.root_path + paths.dataset_path_array[dataset_num] +  "/database_entries_processed.csv";
        std::cout << " reading csv at = " << video_path << std::endl;
        std::ifstream entriesFile(video_path);
        if (entriesFile.fail()) {
            std::cerr << "failed to open file " << video_path << std::endl;
        }
        entriesFile.exceptions(std::ios::badbit);

        std::string line;
        if (!std::getline(entriesFile, line)) {
            // ...then it's an empty file
            std::cerr << "empty file" << std::endl;
        }

        // Read field names, using comma as separator and trimming whitespace
        std::vector<std::string> fields;
        BoBRobotics::strSplit(line, ',', fields);
        std::for_each(fields.begin(), fields.end(), BoBRobotics::strTrim);
        const size_t numFields = fields.size();
        std::cout << " fields = " <<  numFields << std::endl;

        // strings to parse csv
        const char * s_timestamp = "Timestamp [ms]";
        const char * s_x = "X [mm]";
        const char * s_y = "Y [mm]";
        const char * s_z = "Z [mm]";
        const char * s_heading = "Heading [degrees]";
        const char * s_pitch = "Pitch [degrees]";
        const char * s_roll = "Roll [degrees]";
        const char * s_speed = "Speed";
        const char * s_steering = "Steering angle [degrees]";
        const char * s_filename = "Filename";
        const char * s_gps_quality = "GPS quality";
        const char * s_utm_zone = "UTM zone";
        const char * s_fx = "fitted x deg 1";
        const char * s_fy = "fitted y deg 1";
        const char * s_fz = "fitted z deg 1";
        const char * s_gps_h = "gps_h deg 1";
        const char * s_corr_heading = "corrected IMU heading [degrees]";

        // field names
        std::array<const char *, 17> defaultFieldNames{
            s_x, s_y, s_z, s_heading, s_pitch, s_roll, s_speed,
            s_steering, s_filename, s_gps_quality, s_utm_zone,
            s_fx, s_fy, s_fz, s_gps_h, s_corr_heading
        };

        std::cout << "parsing csv file" << std::endl;
         // Read data line by line
        while (std::getline(entriesFile, line)) {
            // Ignore empty lines
            BoBRobotics::strTrim(line);
            if (line.empty()) {
                continue;
            }

                // Use comma as delimiter
            BoBRobotics::strSplit(line, ',', fields);
            BOB_ASSERT(fields.size() == numFields);
            std::for_each(fields.begin(), fields.end(), BoBRobotics::strTrim);

            // Treat empty strings as NaNs (this is how pandas encodes them)
            const auto getDouble = [](const std::string &s) {
                return s.empty() ? NAN : std::stod(s);
            };

            Database_entry entry;
            entry.s_timestamp = float(getDouble(fields[0]));
            entry.s_x = millimeter_t(getDouble(fields[1]));
            entry.s_y = millimeter_t(getDouble(fields[2]));
            entry.s_z = millimeter_t(getDouble(fields[3]));
            entry.s_heading = degree_t(getDouble(fields[4]));
            entry.s_pitch = degree_t(getDouble(fields[5]));
            entry.s_roll = degree_t(getDouble(fields[6]));
            entry.s_speed = float(getDouble(fields[7]));
            entry.s_steering = (getDouble(fields[8]));
            entry.s_filename = fields[9];
            entry.s_gps_quality = int(getDouble(fields[10]));
            entry.s_utm_zone = fields[11];
            entry.s_fx = millimeter_t(getDouble(fields[12]));
            entry.s_fy = millimeter_t(getDouble(fields[13]));
            entry.s_fz = millimeter_t(getDouble(fields[14]));
            entry.s_gps_h = degree_t(getDouble(fields[15]));
            entry.s_corr_heading = degree_t(getDouble(fields[16]));
            entries.push_back(entry);
        }
        std::cout << "csv file successfully read" << std::endl;
        return entries;
    }

};

// route
struct Route {
    std::vector<RouteNode> nodes;
    int num_rotated_views;

    //! gets closest node from outer route - to be done
    RouteNode getClosestNode(Route otherRoute) {
        RouteNode closestNode;
        return closestNode;
    }

    static Route setup(int dataset_num, int num_rotations, bool unwrap, bool createvideo) {

        auto db_entries = CSVReader::loadCSV(dataset_num);
        // read images

        std::cout << "Loading images..." << std::endl;
        VideoReader reader;
        std::vector<cv::Mat> images = reader.readImages(dataset_num, unwrap, createvideo);

        // create route
        Route rc_car_route;
        rc_car_route.num_rotated_views = num_rotations; // rotations
        for (int i = 0; i < db_entries.size();i++) {
        // create route node
            if (db_entries[i].s_utm_zone != "" ) {
                RouteNode node;
                node.x = db_entries[i].s_fx; // to fill with csv data
                node.y = db_entries[i].s_fy;
                node.z = db_entries[i].s_fz;
                node.heading = db_entries[i].s_corr_heading;

                // getting only the frames which have corresponding coordinates
                auto fileName = db_entries[i].s_filename;
                // extracting the number from the filename to match with video frame numbers
                std::stringstream digits;
                for(int i=0;i<fileName.length();i++){
                    if(isdigit(fileName[i])) {
                        digits << fileName[i];
                    }
                }
                int frameNumber;
                digits >> frameNumber;

                node.image = images[frameNumber];
                cv::Mat img1,img2;
                cv::cvtColor(node.image, img1, cv::COLOR_BGR2GRAY);
                img1.convertTo(img2, CV_32F, 1.0 / 255);
                node.image_hash = BoBRobotics::ImgProc::DCTHash::computeHash(img2);
                rc_car_route.nodes.push_back(node);
            }

        }
        std::cout << " route is successfully read " << std::endl;
        return rc_car_route;
    }
};


