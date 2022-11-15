#pragma once
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
#include <sstream>
#include <thread>

#include "video/panoramic.h"
#include "include/common/string.h"
#include "include/common/macros.h"
#include "imgproc/roll.h"
#include "common/stopwatch.h"
#include "third_party/units.h"
#include "gpu_hasher.h"
#include "gpu_dct.h"


using namespace units;
using namespace units::literals;
using namespace units::angle;
using namespace units::length;
using meter_t = units::length::meter_t;
using millimeter_t = units::length::millimeter_t;
using degree_t = units::angle::degree_t;


using namespace BoBRobotics;
using namespace ImgProc;


// setup a navigation route with these classes
struct dataset_paths {
    std::vector<std::string> dataset_path_array;
    std::string root_path = "../../../../../../../../media/tenxlenx/3d_assets/simulation_databases/";
    dataset_paths() {}

};

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
    cv::Mat image;
    int node_number;

    //! Eauclidean distance between two 3d coordinates
    static millimeter_t distance(RouteNode node1, RouteNode node2) {

        using namespace units::math;
        millimeter_t x1 = node1.x;
        millimeter_t y1 = node1.y;
        millimeter_t z1 = node1.z;

        millimeter_t x2 = node2.x;
        millimeter_t y2 = node2.y;
        millimeter_t z2 = node2.z;


        millimeter_t d = sqrt( pow<2>(x2 - x1) + pow<2>(y2 - y1) + pow<2>(z2 - z1) );
        return d;

    }

    static degree_t normalise_angle(degree_t angle) {
        float ang_val = fmod((angle.to<float>() + 180) , 360) - 180;
        return degree_t(ang_val);

    }

    //! gets the angular difference between 2 nodes
    static degree_t angle_distance(RouteNode node1, RouteNode node2, units::angle::degree_t rotation) {
        degree_t angle1 = normalise_angle(node1.heading);
        degree_t angle2 = normalise_angle(node2.heading);
        degree_t rot = normalise_angle(rotation);

        degree_t diff = (angle2 - rot);

        degree_t err = diff - angle1;
        degree_t err_val = normalise_angle(err);



        return err_val;
    }



};


// a csv reader to read database files
struct CSVReader {
    static std::vector<Database_entry> loadCSV(int dataset_num) {
        dataset_paths paths;
        std::vector<Database_entry> entries;
        std::string video_path = paths.root_path + paths.dataset_path_array[dataset_num] +  "/database_entries.csv";
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
            entry.s_timestamp = float(getDouble(fields[9]));
            entry.s_x = millimeter_t(getDouble(fields[0]));
            entry.s_y = millimeter_t(getDouble(fields[1]));
            entry.s_z = millimeter_t(getDouble(fields[2]));
            entry.s_heading = degree_t(getDouble(fields[3]));
            entry.s_pitch = degree_t(getDouble(fields[4]));
            entry.s_roll = degree_t(getDouble(fields[5]));
            entry.s_speed = float(getDouble(fields[7]));
            entry.s_steering = (getDouble(fields[8]));
            entry.s_filename = fields[6];
            entries.push_back(entry);
        }
        std::cout << "csv file successfully read" << std::endl;
        return entries;
    }

    static std::vector<Database_entry> loadCSV(std::string dataset_name) {
        dataset_paths paths;
        std::vector<Database_entry> entries;
        std::string video_path = paths.root_path + dataset_name +  "/database_entries.csv";
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
            entry.s_timestamp = float(getDouble(fields[9]));
            entry.s_x = millimeter_t(getDouble(fields[0]));
            entry.s_y = millimeter_t(getDouble(fields[1]));
            entry.s_z = millimeter_t(getDouble(fields[2]));
            entry.s_heading = degree_t(getDouble(fields[3]));
            entry.s_pitch = degree_t(getDouble(fields[4]));
            entry.s_roll = degree_t(getDouble(fields[5]));
            entry.s_speed = float(getDouble(fields[7]));
            entry.s_steering = (getDouble(fields[8]));
            entry.s_filename = fields[6];
            entries.push_back(entry);
        }
        std::cout << "csv file successfully read" << std::endl;
        return entries;
    }

    // load csv, but user selects the location of the columns in the database
    static std::vector<Database_entry> loadCSV(std::string dataset_name, std::vector<int> csv_columns) {
        dataset_paths paths;
        std::vector<Database_entry> entries;
        std::string video_path = paths.root_path + dataset_name +  "/database_entries.csv";
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
            entry.s_timestamp = float(getDouble(fields[9]));
            entry.s_x = millimeter_t(getDouble(fields[0]));
            entry.s_y = millimeter_t(getDouble(fields[1]));
            entry.s_z = millimeter_t(getDouble(fields[2]));
            entry.s_heading = degree_t(getDouble(fields[3]));
            entry.s_pitch = degree_t(getDouble(fields[4]));
            entry.s_roll = degree_t(getDouble(fields[5]));
            entry.s_speed = float(getDouble(fields[7]));
            entry.s_steering = (getDouble(fields[8]));
            entry.s_filename = fields[6];
            entries.push_back(entry);
        }
        std::cout << "csv file successfully read" << std::endl;
        return entries;
    }

};



// route
class Route {

    public:


    std::vector<RouteNode> nodes;
    int num_rotated_views;

    static std::pair<millimeter_t, degree_t> getMatchErrors(Route &train, RouteNode &test_node,  int match, degree_t rotation_match) {
        millimeter_t position_error = RouteNode::distance(test_node, train.nodes[match]);
        degree_t ang_error = RouteNode::angle_distance(train.nodes[match], test_node, rotation_match);
        return std::make_pair(position_error, ang_error);
    }

    static std::vector<std::vector<meter_t>> route_dist_matrix(Route &train, Route &test) {
        int train_size = train.nodes.size();
        int test_size = test.nodes.size();

        std::vector<std::vector<meter_t>> dist_matrix;
        for (int i = 0; i < train_size; i++) {
            std::vector<meter_t> dist_row;
            for (int j = 0; j < test_size; j++) {
                auto n1 = train.nodes[i];
                auto n2 = test.nodes[j];
                meter_t distance = RouteNode::distance(n1,n2);
                dist_row.push_back(distance);
            }
            dist_matrix.push_back(dist_row);
        }
        return dist_matrix;
    }



    Route() {}




    Route(int dataset_num, int num_rotations, int skipstep, bool unwrap, bool createvideo, cv::Size unwrapRes) {
        auto db_entries = CSVReader::loadCSV(dataset_num);
        std::cout << "Loading images..." << std::endl;
        std::vector<cv::Mat> images;
        dataset_paths paths;

        // create route
        num_rotated_views = num_rotations; // rotations
        for (size_t i = 0; i < db_entries.size()-skipstep;i+=skipstep) {
            // create route node

            RouteNode node;
            node.x = db_entries[i].s_x; // to fill with csv data
            node.y = db_entries[i].s_y;
            node.z = db_entries[i].s_z;
            node.heading = db_entries[i].s_heading;

            // getting only the frames which have corresponding coordinates
            auto fileName = db_entries[i].s_filename;
            std::string img_path = paths.root_path + paths.dataset_path_array[dataset_num] + "/" + fileName;

            //node.image
            auto image = cv::imread(img_path);
            //cv::GaussianBlur(image, image, cv::Size(3, 3), 0);
            //cv::resize(image, image, unwrapRes,cv::INTER_CUBIC);
            node.image = image;


            //cv::Mat img1,img2;
            //cv::cvtColor(node.image, img1, cv::COLOR_BGR2GRAY);
           // cv::equalizeHist(img1,img1);

            //img1.convertTo(img2, CV_32F, 1.0 / 255);
            nodes.push_back(node);


        }
        std::cout << " route is successfully read " << std::endl;

    }

    Route(std::string dataset_path, int num_rotations, int skipstep, bool unwrap, bool createvideo, cv::Size unwrapRes, bool hist_equalize, bool smoothing, std::vector<int> csv_columns ) {
        auto db_entries = CSVReader::loadCSV(dataset_path, csv_columns);
        std::cout << "Loading images..." << std::endl;
        //VideoReader reader;
        //std::vector<cv::Mat> images = reader.readImages(dataset_num, unwrapRes);
        std::vector<cv::Mat> images;
        dataset_paths paths;


        // create route
        num_rotated_views = num_rotations; // rotations
        for (size_t i = 0; i < db_entries.size()-skipstep;i+=skipstep) {
            // create route node

            RouteNode node;
            node.x = db_entries[i].s_x; // to fill with csv data
            node.y = db_entries[i].s_y;
            node.z = db_entries[i].s_z;
            node.heading = db_entries[i].s_heading;

            // getting only the frames which have corresponding coordinates
            auto fileName = db_entries[i].s_filename;
            std::string img_path = paths.root_path + dataset_path + "/" + fileName;

            //node.image
            auto image = cv::imread(img_path);

            //cv::GaussianBlur(image, image, cv::Size(5, 5), 0);
            //cv::resize(image, image, unwrapRes,cv::INTER_CUBIC);
            node.image = image;


            //cv::Mat img1,img2;
            //cv::cvtColor(node.image, img1, cv::COLOR_BGR2GRAY);
            //cv::equalizeHist(img1,img1);

            //img1.convertTo(img2, CV_32F, 1.0 / 255);
            nodes.push_back(node);


        }
        std::cout << " route is successfully read " << std::endl;

    }


};


