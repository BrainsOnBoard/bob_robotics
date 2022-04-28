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


using namespace BoBRobotics;
using namespace ImgProc;


// setup a navigation route with these classes

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

    //! Eauclidean distance between two 3d coordinates
    static millimeter_t distance(RouteNode node1, RouteNode node2) {

        using namespace units::math;
        auto x1 = node1.x;
        auto y1 = node1.y;
        auto z1 = node1.z;

        auto x2 = node2.x;
        auto y2 = node2.y;
        auto z2 = node2.z;

        millimeter_t d = sqrt( pow<2>(x2 - x1) + pow<2>(y2 - y1) + pow<2>(z2 - z1) );
        return d;

    }

    //! gets the angular difference between 2 nodes
    static degree_t angle_distance(RouteNode node1, RouteNode node2) {
        auto angle1 = node1.heading;
        auto angle2 = node2.heading;

        degree_t delta_theta = (angle1 > angle2) * (360_deg - angle2 - angle1)
              + (angle2 > angle1) * (angle2 - angle1);

        return delta_theta;
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

class HashMatrix
{
    public:

    size_t index( int x, int y ) const { return x + m_width * y; }
    int m_width;  // height of the dataset ( number of data elements)
    int m_height; // width of dataset ( number of rotate views )
    std::vector<cv::Mat> images;
    std::vector<std::bitset<64> > m_matrix; // training examples with pre-calc rotations


    //! get a difference hash vector form the best views ( 1rotation/ element)
    std::vector<int> getBestHashRotationDists(std::vector<int> matrix, bool norm = true, int norm_window = 5) {
        std::vector<int> bestRotVector;
        for (size_t i=0; i< matrix.size()/m_width; i++) {
            int min = 255;
            for (int j = 0; j < m_width; j++ ) {
                auto current_element = matrix[i*m_width+j];
                if (current_element < min) {
                    min = current_element;
                }
            }
            // save best rotation's hash distance
            bestRotVector.push_back(min);
        }

        // normalising





        return bestRotVector;
    }

    static void normalise_n(std::vector<int> &vector, int norm_window) {


        std::vector<int> vectorC(vector.size());
        for (int j = 0; j < vector.size(); j++) {

            std::vector<int> window;

            for (int i = 0; i < norm_window; i++) {

                if (j-i >0) {
                    window.push_back(vector[j-i]);
                }
                if (j+i < vector.size()) {
                    window.push_back(vector[j+i]);
                }

            }

            double sum = std::accumulate(window.begin(), window.end(), 0.0);
            double mean = sum / double(window.size());
            double sq_sum = std::inner_product(window.begin(), window.end(), window.begin(), 0.0);
            double stdev = std::sqrt(sq_sum / float(window.size()) - mean * mean);
            vectorC[j] = (vector[j] - mean) / stdev;
        }
        vector = vectorC;


    }

    //! gets the matrix object
    std::vector<std::bitset<64>> getMatrix() { return m_matrix; }

    //! get the image from database with given rotation
    cv::Mat getImage( int node_num, int rotation) {
        cv::Mat img = images[ node_num ];
        std::vector<cv::Mat> img_rotations;
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        HashMatrix::getHashRotations(img, m_width, img_rotations );
        return img_rotations[rotation];
    }

    HashMatrix() {}

    //! creates a  hash matrix with a route
    HashMatrix(std::vector<RouteNode> nodes, int numRotations) {


        this->m_width = numRotations;
        this->m_height = nodes.size();


        for (size_t i = 0; i < nodes.size(); i++) {
            cv::Mat img1 = nodes[i].image; // get current image
            cv::Mat img_gray;
            images.push_back(img1);

            std::vector<cv::Mat> mat;
            cv::cvtColor(img1, img_gray, cv::COLOR_BGR2GRAY);
            auto hash_rotations = HashMatrix::getHashRotations(img_gray, numRotations, mat); // gets all the rotations (and hashes)
            for (size_t j = 0; j < hash_rotations.size(); j++) {
                m_matrix.push_back(hash_rotations[j] );
            }

        }
    }

    //! gets rotations
    static std::vector<std::bitset<64>> getHashRotations(cv::Mat image, int totalRotations, std::vector<cv::Mat> &img_rotations) {
        // rotate member variable matrix

        auto image_width = image.size().width;
        auto pixel_to_rot = 360 / image_width;
        std::vector<std::bitset<64>> rotations;
        cv::Mat rolledImage;

        cv::Mat rolledImageFormatted;
        for (int i = 0; i < totalRotations;  i++) {
            ImgProc::roll(image, rolledImage, i * pixel_to_rot);
            img_rotations.push_back(rolledImage);
            rolledImage.convertTo(rolledImageFormatted, CV_32F, 1.0 / 255);
            rotations.push_back(DCTHash::computeHash(rolledImageFormatted));
        }

        return rotations;
    }

    //! hash all values in matrix with given hash
    static std::vector<int> calculateHashValues(std::bitset<64> hash, const std::vector<std::bitset<64>> hashMatrix) {
        std::vector<int> differenceMatrix;
        for (size_t i =0; i < hashMatrix.size(); i++) {
            differenceMatrix.push_back( DCTHash::distance(hashMatrix[i], hash) );
        }
        return differenceMatrix;
    }

    //! gets the minimum element of a matrix and it's 2d indices
    static void argmin_matrix(std::vector<int> &matrix, int width, int &min_col, int &min_row, int &min_value) {
        int min = 100000; // init with big number
        int index_min = 0;
        for (size_t i=0; i< matrix.size(); i++) {
            if (matrix[i] < min) {
                min = matrix[i];
                index_min = i;
            }
        }
        min_value = min;
        min_col = index_min % width;    // % is the "modulo operator", the remainder of i / width;
        min_row = index_min / width;    // where "/" is an integer division
    }

    //! matches an images hash against a database (single match)
    static std::pair<int,int> getSingleMatch(std::bitset<64> hash, HashMatrix hashmat, int &min_value, int width) {
        int min_col, min_row;
        std::vector<int> differenceMatrix = HashMatrix::calculateHashValues(hash,hashmat.getMatrix());
        HashMatrix::argmin_matrix(differenceMatrix , width, min_col, min_row, min_value);
        std::pair<int,int> mat_position({min_col, min_row});
        return mat_position;
    }
};



// route
class Route {

    public:


    std::vector<RouteNode> nodes;
    int num_rotated_views;
    HashMatrix m_hash_matrix;

    //! gets closest node from outer route and it's distance
    static RouteNode getClosestNode(Route route_train, Route route_test, int node_to_find, millimeter_t &distance ) {

        // get test node by index
        RouteNode toFind = route_test.nodes[node_to_find];
        // go through train values to pick closest node
        auto train_nodes = route_train.nodes;
        millimeter_t min_dist = 1000_m;
        int min_index = 0;
        for ( size_t i = 0; i < train_nodes.size();i++) {
            auto curr_dist = RouteNode::distance(train_nodes[i], toFind);
            if (curr_dist < min_dist) {
                min_dist = curr_dist;
                min_index = i;
            }
        }
        distance = min_dist;
        RouteNode retNode = route_test.nodes[min_index];
        return retNode;
    }

    Route() {}

    Route(int dataset_num, int num_rotations, int skipstep, bool unwrap, bool createvideo, cv::Size unwrapRes) {
        auto db_entries = CSVReader::loadCSV(dataset_num);
        std::cout << "Loading images..." << std::endl;
        VideoReader reader;
        std::vector<cv::Mat> images = reader.readImages(dataset_num, unwrapRes);

        // create route
        num_rotated_views = num_rotations; // rotations
        for (size_t i = 0; i < db_entries.size()-skipstep;i+=skipstep) {
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
                for(size_t i=0;i<fileName.length();i++){
                    if(isdigit(fileName[i])) {
                        digits << fileName[i];
                    }
                }
                int frameNumber;
                digits >> frameNumber;

                node.image = images[frameNumber];
                cv::Mat img1,img2;
                cv::cvtColor(node.image, img1, cv::COLOR_BGR2GRAY);
                cv::equalizeHist(img1,img1);
                img1.convertTo(img2, CV_32F, 1.0 / 255);


                node.image_hash = BoBRobotics::ImgProc::DCTHash::computeHash(img2);

                nodes.push_back(node);
            }

        }
        std::cout << " route is successfully read " << std::endl;

    }

    HashMatrix getHashMatrix() { return m_hash_matrix; }

    void set_hash_matrix(HashMatrix matrix) {
        m_hash_matrix = matrix;
    }

};


