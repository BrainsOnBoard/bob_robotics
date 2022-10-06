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
#include "imgproc/dct_hash.h"
#include "imgproc/roll.h"
#include "common/stopwatch.h"
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
struct dataset_paths {
    std::vector<std::string> dataset_path_array;
    std::string root_path = "../../../../../simulation_databases/";
    dataset_paths() {
        std::string dataset0 = "route1_red";
        std::string dataset1 = "route2_cloudy";
        std::string dataset2 = "route3_partly_cloudy";
        std::string dataset3 = "route4_partly_cloudy2";
        dataset_path_array.push_back(dataset0);
        dataset_path_array.push_back(dataset1);
        dataset_path_array.push_back(dataset2);
        dataset_path_array.push_back(dataset3);
    }
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
    std::bitset<64> image_hash;
    cv::Mat image;
    int node_number;

    //! Eauclidean distance between two 3d coordinates
    static meter_t distance(RouteNode node1, RouteNode node2) {

        using namespace units::math;
        auto x1 = node1.x;
        auto y1 = node1.y;
        auto z1 = node1.z;

        auto x2 = node2.x;
        auto y2 = node2.y;
        auto z2 = node2.z;

        meter_t d = sqrt( pow<2>(x2 - x1) + pow<2>(y2 - y1) + pow<2>(z2 - z1) );
        return d;

    }

    //! gets the angular difference between 2 nodes
    static degree_t angle_distance(RouteNode node1, RouteNode node2, units::angle::degree_t rotation) {
        auto angle1 = node1.heading;
        auto angle2 = (node2.heading - rotation);

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

};

class HashMatrix
{
    public:

    size_t index( int x, int y ) const { return x + m_width * y; }
    int m_width;  // height of the dataset ( number of data elements)
    int m_height; // width of dataset ( number of rotate views )
    std::vector<cv::Mat> images;
    std::vector<std::bitset<64> > m_matrix; // training examples with pre-calc rotations

    int getWidth() const { return m_width; }

    //! get a difference hash vector form the best views ( 1rotation/ element)
    static std::pair<std::vector<int>, std::vector<int>> getBestHashRotationDists(std::vector<int> &matrix, int width, std::vector<int> &bestRotationDists) {
        std::vector<int >bestRotVector;
        std::vector<int> bestRotationAngles;
        std::vector<int> min_indices;

        int min_index = 0;
        for (size_t i=0; i< matrix.size()/width; i++) {
            int min = 255;
            int bestRot = 0;

            for (int j = 0; j < width; j++ ) {
                auto current_element = matrix[i*width+j];
                if (current_element <= min) {
                    min = current_element;
                    bestRot = j;
                    min_index = i;

                }

            }
            // save best rotation's hash distance

            bestRotVector.push_back(min);
            bestRotationAngles.push_back(bestRot);
        }

        for (int r = 0; r < width; r++) {
            auto rot_element = matrix[min_index * width + r];
            bestRotationDists.push_back(rot_element);
        }

        return {bestRotVector, bestRotationAngles};
    }



    static void normalise_n(std::vector<int> &vector, int norm_window) {
        for (int j = 0; j < vector.size(); j++) {
            std::vector<int> window(norm_window);
            for (int i = 0; i < norm_window; i++) {

                if (j-i >=0) {
                    window.push_back(vector[j-i]);
                } else {
                    window.push_back(0); // 0 padding
                }

                if (j+i < vector.size()) {
                    window.push_back(vector[j+i]);
                } else {
                    window.push_back(0); // 0 padding
                }
            }

            double sum = std::accumulate(window.begin(), window.end(), 0.0);
            double mean = sum / double(window.size());
            double sq_sum = std::inner_product(window.begin(), window.end(), window.begin(), 0.0);
            double stdev = std::sqrt(sq_sum / float(window.size()) - mean * mean);
            vector[j] = (vector[j] - mean) / stdev;
        }
        vector = vector;
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

            std::vector<cv::Mat> mat(numRotations);
            cv::cvtColor(img1, img_gray, cv::COLOR_BGRA2GRAY);
            auto hash_rotations = HashMatrix::getHashRotations(img_gray, numRotations, mat); // gets all the rotations (and hashes)

            for (size_t j = 0; j < hash_rotations.size(); j++) {
                m_matrix.push_back(hash_rotations[j] );
            }

        }



    }

    static void rotation_thread(cv::Mat image, int rotate_by, cv::Mat &rolledImage, std::bitset<64> &rotation) {

        cv::Mat rolledImageFormatted;
        ImgProc::rollLeft(image, rolledImage, rotate_by);
        rolledImage.convertTo(rolledImageFormatted, CV_32F, 1.0 / 255);
        rotation = DCTHash::computeHash(rolledImageFormatted);
    }

    //! gets rotations
    static std::vector<std::bitset<64>> getHashRotations(cv::Mat image, int totalRotations, std::vector<cv::Mat> &img_rotations)  {
        // rotate member variable matrix
        auto image_width = image.size().width;
        auto pixel_to_rot = (float)image_width / (float)totalRotations;
        std::vector<std::bitset<64>> rotations(totalRotations);
        std::vector<std::thread> threads;
        for (int i = 0; i < totalRotations; i++) {
            int rotate_by = int(i * (int)pixel_to_rot);
            std::thread thr(rotation_thread, std::ref(image), rotate_by, std::ref(img_rotations[i]), std::ref(rotations[i]));
            threads.push_back(std::move(thr));
        }
        // synch threads
        for (auto&& t : threads) {
            t.join();
        }



        return rotations;
    }

    //! hash all values in matrix with given hash
    static std::vector<int> calculateHashValues(std::bitset<64> hash, std::vector<std::bitset<64>> hashMatrix) {
        std::vector<int> differenceMatrix;
        for (size_t i =0; i < hashMatrix.size(); i++) {
            differenceMatrix.push_back( DCTHash::distance(hashMatrix[i], hash) );
        }
        return differenceMatrix;
    }

    //! gets the minimum element of a matrix and it's 2d indices
    template <typename T>
    static void argmin_matrix(std::vector<T> &matrix, int width, int &min_col, int &min_row, T &min_value) {
        T min = 1000000000; // init with big number
        int index_min = 0;
        for (size_t i=0; i< matrix.size(); i++) {
            if (matrix[i] <= min) {
                min = matrix[i];
                index_min = i;
            }
        }
        min_value = min;
        min_col = fmod(index_min, width);    // % is the "modulo operator", the remainder of i / width;
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

    // converts hashmat to C pointer style (for CUDA)
    void getHashMatUL(unsigned long long int *hashMat) {
       
        for (int i = 0; i < m_matrix.size(); i++) {
            hashMat[i] = m_matrix[i].to_ullong();
        }
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
            std::string img_path = paths.root_path + paths.dataset_path_array[dataset_num] + "/" + fileName;

            //node.image
            auto image = cv::imread(img_path);
            cv::resize(image, image, unwrapRes);
            node.image = image;


            cv::Mat img1,img2;
            cv::cvtColor(node.image, img1, cv::COLOR_BGR2GRAY);
            //cv::equalizeHist(img1,img1);

            img1.convertTo(img2, CV_32F, 1.0 / 255);


            node.image_hash = BoBRobotics::ImgProc::DCTHash::computeHash(img2);
            nodes.push_back(node);


        }
        std::cout << " route is successfully read " << std::endl;

    }

    static std::pair<int,int> pixel_distance_matrix(cv::Mat current_image, Route training_route, int num_rotations) {
        std::vector<cv::Mat> rotated_images(num_rotations);
        HashMatrix::getHashRotations(current_image, num_rotations, rotated_images);

        std::vector<double> dist_mat;
        for (int j = 0; j < training_route.nodes.size(); j++) {

            cv::Mat training_img = training_route.nodes[j].image;
            cv::cvtColor(training_img, training_img, cv::COLOR_BGRA2GRAY);
            for (int i = 0; i < rotated_images.size(); i++) {
                cv::Mat current_rot = rotated_images[i];
                double dist = cv::norm(training_img, current_rot, cv::NORM_L2SQR);
                dist_mat.push_back(dist);
            }
        }

        int min_col,min_row;
        double min_value;
        HashMatrix::argmin_matrix(dist_mat, num_rotations, min_col, min_row, min_value);
        return {min_row,min_col};
    }

    HashMatrix getHashMatrix() { return m_hash_matrix; }

    

    void set_hash_matrix(HashMatrix matrix) {
        m_hash_matrix = matrix;
    }

};


