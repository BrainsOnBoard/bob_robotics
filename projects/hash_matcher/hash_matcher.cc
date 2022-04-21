
// includes
# include <vector>
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
#include "../include/imgproc/dct_hash.h"
#include "video_unwrapper.h"
#include "../include/common/stopwatch.h"
#include "../third_party/units.h"
#include "../include/video/panoramic.h"
#include "../include/imgproc/roll.h"
#include "../include/common/string.h"
#include "../include/common/macros.h"
#include <opencv2/opencv.hpp>

using namespace units::literals;
using namespace units::angle;
using namespace units::length;
using meter_t = units::length::meter_t;
using millimeter_t = units::length::millimeter_t;
using degree_t = units::angle::degree_t;

// a struct containing the names of the datasets for convenience
struct dataset_paths {
    std::vector<std::string> dataset_path_array;
    std::string root_path = "../../../../rc_car_image_databases/rc_car_big/";
    dataset_paths() {

        std::string dataset0  = "20210303_150314";
        std::string dataset1  = "20210303_153749";
        std::string dataset2  = "20210308_122125";
        std::string dataset3  = "20210308_124836";
        std::string dataset4  = "20210322_155544";
        std::string dataset5  = "20210322_170743";
        std::string dataset6  = "20210414_151116";
        std::string dataset7  = "20210420_135721";
        std::string dataset8  = "20210420_141940";
        std::string dataset9  = "20210422_134815";
        std::string dataset10 = "20210426_162152";
        std::string dataset11 = "20210426_164219";
        std::string dataset12 = "20210511_151514";
        std::string dataset13 = "20210511_153933";
        std::string dataset14 = "20210525_141844";

        dataset_path_array.push_back(dataset0);
        dataset_path_array.push_back(dataset1);
        dataset_path_array.push_back(dataset2);
        dataset_path_array.push_back(dataset3);
        dataset_path_array.push_back(dataset4);
        dataset_path_array.push_back(dataset5);
        dataset_path_array.push_back(dataset6);
        dataset_path_array.push_back(dataset7);
        dataset_path_array.push_back(dataset8);
        dataset_path_array.push_back(dataset9);
        dataset_path_array.push_back(dataset10);
        dataset_path_array.push_back(dataset11);
        dataset_path_array.push_back(dataset12);
        dataset_path_array.push_back(dataset13);
        dataset_path_array.push_back(dataset14);
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

using namespace std::literals;
using namespace units::length;
using namespace units::time;
using namespace BoBRobotics;
using namespace ImgProc;

// node of route
struct RouteNode {
    millimeter_t x;
    millimeter_t y;
    millimeter_t z;
    degree_t heading;
    std::bitset<64> image_hash;
    cv::Mat image;
};

// route
struct Route {
    std::vector<RouteNode> nodes;
    int num_rotated_views;
};


// hash matrix
class HashMatrix
{

    public:

    size_t index( int x, int y ) const { return x + m_width * y; }
    int m_width;
    int m_height;
    std::vector<cv::Mat> images;
    std::vector<std::bitset<64> > m_matrix;
    int *m_return_matrix;
    Route route;


    std::vector<int> getBestHashRotationDists(std::vector<int> matrix) {
        std::vector<int> bestRotVector;
        for (int i=0; i< matrix.size()/m_width; i++) {
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
        return bestRotVector;
    }


    std::vector<std::bitset<64>> getMatrix() { return m_matrix; }


    // get the image from database with given rotation
    cv::Mat getImage( int node_num, int rotation) {
        cv::Mat img = images[ node_num ];
        std::vector<cv::Mat> img_rotations;
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        HashMatrix::getHashRotations(img, m_width, img_rotations );
        return img_rotations[rotation];
    }

    HashMatrix() {}

    //! creates a  hash matrix with a route
    HashMatrix(Route &route) {

        int numRotations = route.num_rotated_views;
        this->m_width = numRotations;
        this->m_height = route.nodes.size();
        this->route = route;

        for (int i = 0; i < route.nodes.size(); i++) {
            cv::Mat img1 = route.nodes[i].image; // get current image
            cv::Mat img_gray;
            images.push_back(img1);

            std::vector<cv::Mat> mat;
            cv::cvtColor(img1, img_gray, cv::COLOR_BGR2GRAY);
            auto hash_rotations = HashMatrix::getHashRotations(img_gray, numRotations, mat); // gets all the rotations (and hashes)
            for (int j = 0; j < hash_rotations.size(); j++) {
                m_matrix.push_back(hash_rotations[j] );
            }

        }
    }



    // gets rotations
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

    // hash all values in matrix with given hash
    static std::vector<int> calculateHashValues(std::bitset<64> hash, const std::vector<std::bitset<64>> hashMatrix) {
        std::vector<int> differenceMatrix;
        for (int i =0; i < hashMatrix.size(); i++) {
            differenceMatrix.push_back( DCTHash::distance(hashMatrix[i], hash) );
        }
        return differenceMatrix;
    }

    // to fix
    static void argmin_matrix(std::vector<int> &matrix, int width, int height, int &min_col, int &min_row, int &min_value) {
        int min = 100000; // init with big number
        int index_min = 0;
        for (int i=0; i< matrix.size(); i++) {
            if (matrix[i] < min) {
                min = matrix[i];
                index_min = i;
            }
        }
        min_value = min;
        min_col = index_min % width;    // % is the "modulo operator", the remainder of i / width;
        min_row = index_min / width;    // where "/" is an integer division
    }
};

// reading a video file
class VideoReader {
    public:

    cv::Size unwrapRes;
    unsigned int outputScale;
    VideoUnwrapper uw;


    VideoReader() {
        unwrapRes = cv::Size(90, 25);
        outputScale = 10;
    }



    // read video to images
    std::vector<cv::Mat> readImages(int dataset_num, bool unwrap,bool createfile, cv::Size unwrapRes = cv::Size(90,25)) {
        dataset_paths paths;
        std::string video_path = paths.root_path + paths.dataset_path_array[dataset_num] +  "/" + paths.dataset_path_array[dataset_num] + ".mp4";


        if (!unwrap) {

            if (createfile) {
                uw.unwrapMP4(video_path, unwrapRes, "pixpro_usb");
                video_path = paths.root_path + paths.dataset_path_array[dataset_num] +  "/unwrapped_" + paths.dataset_path_array[dataset_num] + ".mp4";
            } else {
                video_path = paths.root_path + paths.dataset_path_array[dataset_num] +  "/unwrapped_" + paths.dataset_path_array[dataset_num] + ".mp4";
            }
        }

        cv::VideoCapture cap(video_path);


        std::vector<cv::Mat> unwrapped_frames;

        if( !cap.isOpened() )
            throw "Error when reading steam_avi";

        const cv::Size cameraRes(1440,1440);
        const unsigned int outputScale = 10;

        // Create panoramic camera and suitable unwrapper
        OpenCVUnwrap360 unwrapper(cameraRes,unwrapRes, "pixpro_usb");

         // Create images
        cv::Mat originalImage(cv::Size(1440,1440), CV_8UC3);

        for( ; ; ) {
            cv::Mat frame;
            cap >> frame;
            if(frame.empty())
                break;

            if (unwrap) {
                cv::Mat outputImage(unwrapRes, CV_8UC3);
                unwrapper.unwrap(frame, outputImage);
                unwrapped_frames.push_back(outputImage);

            } else {

                unwrapped_frames.push_back(frame);
            }

        }

        return unwrapped_frames;
    }
};

Route setup(int dataset_num, int num_rotations, bool unwrap, bool createvideo) {

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
            node.image_hash = DCTHash::computeHash(img2);
            rc_car_route.nodes.push_back(node);
        }

    }
    std::cout << " route is successfully read " << std::endl;
    return rc_car_route;
}

class DTHW {

    HashMatrix m_long_sequence;
    std::deque<std::bitset<64>> m_short_sequence;
    std::deque<std::vector<int>> m_cost_matrix;
    std::deque<std::vector<int>> m_accumulated_cost_matrix;
    int m_current_sequence_size = 0;
    int m_sequence_limit = 50;
    bool m_genP = true;

    public:

    DTHW(HashMatrix long_sequence) {
        this->m_long_sequence = long_sequence;
    }

    // add to sequence - if length reached, oldest element is removed
    void addToShortSequence(std::bitset<64> hashValue,int sequence_size) {
        m_sequence_limit = sequence_size;
        if (m_short_sequence.size() < sequence_size) {
            m_short_sequence.push_back(hashValue);
        } else {
            m_short_sequence.pop_front();
            m_short_sequence.push_back(hashValue);

        }
        m_current_sequence_size = m_short_sequence.size();

    }


    std::deque<std::vector<int>> calculate_cost_matrix(std::deque<std::bitset<64>> short_sequence, HashMatrix &h_matrix) {
        std::deque<std::vector<int>> costMatrix;

        for (int i = 0; i < short_sequence.size(); i++) {
            std::vector<int> differenceMatrix = HashMatrix::calculateHashValues(short_sequence[i],h_matrix.getMatrix());
            std::vector<int> row_dists = h_matrix.getBestHashRotationDists(differenceMatrix);
            costMatrix.push_back(row_dists);
        }
        m_cost_matrix = costMatrix;;
        return costMatrix;
    }

    std::vector<int> calculateNewRowDistances(std::bitset<64> last_hash_val, HashMatrix &matrix) {
        auto hashmat = matrix.getMatrix();
        std::vector<int> differenceMatrix = HashMatrix::calculateHashValues(last_hash_val,hashmat);
        std::vector<int> new_row = matrix.getBestHashRotationDists(differenceMatrix); // 1 row in cost mat
        return new_row;
    }

    // sequence logic
    std::vector<std::pair<int,int>> getBestSequence(std::deque<std::bitset<64>> short_sequence, HashMatrix &hmat) {


        auto C = m_cost_matrix;

        auto D = m_accumulated_cost_matrix;
        if (D.empty() || m_genP) { // init D mat

             // init cost mat
            C = calculate_cost_matrix(short_sequence, m_long_sequence);
            D = calculate_accumulated_cost_matrix(m_cost_matrix);
            m_accumulated_cost_matrix = D;
            m_genP = false;
        } else { // append row if exists
            D = appendRowToD( calculateNewRowDistances (short_sequence.back(), hmat));
        }



        auto P = calculateOptimalWarpingPath(D); // first is last match - what we want
        if (P[0].second == 0) { // 0 should(?) be error - very hacky
            m_genP = true;
        }

        return P;
    }

    int getBestMatch(HashMatrix &hmat) {
        int r_size = hmat.getMatrix().size();
        if (m_current_sequence_size >= m_sequence_limit) {
            auto P = getBestSequence(m_short_sequence, hmat );
           //int match_index =  P[P.size()-1].second;
            int match_index =  P[0].second;// + m_sequence_limit; // adding the size to get the correct match (seq start + len)
           // if (match_index >= r_size) {
           //     match_index = r_size -1;
           // }
            return match_index;
        } else {
            std::vector<int> differenceMatrix = HashMatrix::calculateHashValues(m_short_sequence.back(),m_long_sequence.getMatrix());
            int min_col, min_row, min_value;
            HashMatrix::argmin_matrix(differenceMatrix , 60, differenceMatrix.size(), min_col, min_row, min_value) ;
            return min_row;
        }
        return 0;

    }

    //! appends a row to D ----  to debug
    std::deque<std::vector<int>> appendRowToD(std::vector<int> cost_row) {
        auto D = m_accumulated_cost_matrix;
        std::vector<int> D_row(cost_row.size());
        auto C = m_cost_matrix;
        C.push_back(cost_row); // append C mat with  new cost row
        C.pop_front(); // remove first
        D.push_back(D_row); // push new empty row
        D.pop_front(); //  remove first

        // copy first element of Cost matrix row
        int N = D.size()-1;
        int M = cost_row.size()-1;

        // add 1 to cum sum
        D[N][0] = D[N-1][0] + C[0][0];

        for (int j = 1; j < M+1; j++) {
            int up = D[N-1][j]; // up
            int left = D[N][j-1]; // left
            int upper_left = D[N-1][j-1]; // upper left
            std::vector<int> squares({up,left,upper_left});
            int min_val = *std::min_element( std::begin(squares), std::end(squares) );
            D.back()[j] = C.back()[j] + min_val;
        }
        m_accumulated_cost_matrix = D;
        m_cost_matrix = C;
        return D;
    }


    std::deque<std::vector<int>> calculate_accumulated_cost_matrix(std::deque<std::vector<int>> C) {

        int N, M;
        N = C.size();
        M = C[0].size();

        std::deque<std::vector<int>> D(C.size(),std::vector<int>(C[0].size())); // accumulated cost matrix
        std::vector<int> cum_sum(C.size());
        std::vector<int> first_col;
         // cumulative sum of first column
        for  (int  i = 0; i < C.size(); i++) {
            first_col.push_back(C[i][0]);
        }
        std::partial_sum(first_col.begin(), first_col.end(), cum_sum.begin(), std::plus<int>());
        for (int i =0;  i < D.size(); i++) {
            D[i][0] = cum_sum[i];
        }


        // copy first row of Cost matrix
        for (int i = 0; i < D[0].size(); i++) {
            D[0][i] = C[0][i]; // row 0 of D = C
        }

        for (int i = 1; i < N; i++) {
            for (int j = 1; j < M; j++) {
                int up = D[i-1][j]; // up
                int left = D[i][j-1]; // left
                int upper_left = D[i-1][j-1]; // upper left
                std::vector<int> squares({up,left,upper_left});
                int min_val = *std::min_element( std::begin(squares), std::end(squares) );
                //D[n, m] = C[n, m] + min(D[n-1, m], D[n, m-1], D[n-1, m-1])
                D[i][j] = C[i][j] + min_val;
            }
        }
        m_accumulated_cost_matrix = D;
        return D;
    }


    std::vector<std::pair<int,int> > calculateOptimalWarpingPath(std::deque<std::vector<int>> D) {
        int N = D.size(); // row size
        int M = D[0].size(); // col size
        int n = N -1;
        int m = -1;

        int min_index = 0;
        int minVal = 1000;
        auto curr_row = D[N-1];
        for (int i = 0; i < curr_row.size(); i++) {  // m = D[N - 1, :].argmin()
            if (curr_row[i] < minVal) {
                minVal = curr_row[i];
                min_index = i;
            }
        }
        m = min_index; // best value of last
        std::cout << std::endl;

        std::vector<std::pair<int,int>> P; // path
        P.push_back({n,m}); // first node

        while (n > 0) {
            std::pair<int,int> path_node;
            if ( m == 0) {
                path_node.first = n-1;
                path_node.second = 0;
            } else {
                std::vector<int> squares({D[n-1][m-1],D[n-1][m],D[n][m-1]});
                int val = *std::min_element( std::begin(squares), std::end(squares) );

                if (val == D[n-1][m-1]) { // if upper left

                    path_node.first = n-1;
                    path_node.second = m-1;
                }
                else if ( val == D[n-1][m]) { // if up

                    path_node.first = n-1;
                    path_node.second = m;
                }
                else {
                     // if left
                    path_node.first = n;
                    path_node.second = m-1;
                }
            }
            P.push_back(path_node);
            n = path_node.first;
            m = path_node.second;
        }

        return P;
    }
};


int main(int argc, char **argv) {

    Stopwatch watchGen;
    watchGen.start();
    auto route1 = setup(0,60, false, false);
    auto route2 = setup(1,60,false, false);

    // setup Hash matrix with route
    HashMatrix hashmat1(route1); // create a matrix with rotations
    HashMatrix hashmat2(route2); // create a matrix with rotations

    int height = route1.nodes.size();

    std::cout << " video reading, unwrapping, hashing took "
              <<  static_cast<second_t>(watchGen.elapsed()).value()
              << " seconds\n";


    Stopwatch watchGen1;
    watchGen1.start();


    DTHW sequence_matcher(hashmat1); // init sequence matcher with training matrices


    int seq_length = 9;
    for (int h = 0; h < route2.nodes.size(); h++) {
        auto hash = route2.nodes[h].image_hash; // current hash of test set
        int min_value;
        int min_col;
        int min_row;

        sequence_matcher.addToShortSequence(hash,seq_length);

        int seq_index = sequence_matcher.getBestMatch(hashmat1);
        std::vector<int> differenceMatrix = HashMatrix::calculateHashValues(hash,hashmat1.getMatrix());
        std::vector<int> bestRots = hashmat1.getBestHashRotationDists(differenceMatrix); // 1 row in cost mat
        HashMatrix::argmin_matrix(differenceMatrix , 60, height, min_col, min_row, min_value) ;
        std::cout << " current = " << h << " single match " <<  min_row << " seq match " << seq_index << " hash " << hash <<  std::endl;
        cv::imshow("current", route2.nodes[h].image);
        cv::imshow("current match", route1.nodes[min_row].image);
        cv::imshow("current seq match", route1.nodes[seq_index].image);
        cv::waitKey(1);
    }
    //HashMatrix::argmin_matrix(hashmat1.calculateHashValues(hash) , 60, height, min_col, min_row, min_value) ;
    //std::cout << " min value " << min_value << " min col " << min_col <<  " min row " << min_row << std::endl;
    std::cout << " hashing the database took "
              <<  static_cast<second_t>(watchGen1.elapsed()).value()
              << " seconds\n";

    return 0;
}



