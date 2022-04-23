
#include "route_setup.h"

using namespace std::literals;
using namespace units::length;
using namespace units::time;
using namespace BoBRobotics;
using namespace ImgProc;

//! hash matrix
class HashMatrix
{
    public:

    size_t index( int x, int y ) const { return x + m_width * y; }
    int m_width;  // height of the dataset ( number of data elements)
    int m_height; // width of dataset ( number of rotate views )
    std::vector<cv::Mat> images;
    std::vector<std::bitset<64> > m_matrix; // training examples with pre-calc rotations
    Route route;

    //! get a difference hash vector form the best views ( 1rotation/ element)
    std::vector<int> getBestHashRotationDists(std::vector<int> matrix) {
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
        return bestRotVector;
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
    HashMatrix(Route &route) {

        int numRotations = route.num_rotated_views;
        this->m_width = numRotations;
        this->m_height = route.nodes.size();
        this->route = route;

        for (size_t i = 0; i < route.nodes.size(); i++) {
            cv::Mat img1 = route.nodes[i].image; // get current image
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

//! hash sequence matcher
class DTHW {

    private:
    HashMatrix m_long_sequence;
    std::deque<std::bitset<64>> m_short_sequence;
    std::deque<std::vector<int>> m_cost_matrix;
    std::deque<std::vector<int>> m_accumulated_cost_matrix;
    unsigned int m_roll_step;
    unsigned int m_current_sequence_size = 0;
    unsigned m_sequence_limit = 50;
    bool m_genP = true;

    // calculate C matrix
    std::deque<std::vector<int>> calculate_cost_matrix(std::deque<std::bitset<64>> short_sequence, HashMatrix &h_matrix) {
        std::deque<std::vector<int>> costMatrix;

        for (size_t i = 0; i < short_sequence.size(); i++) {
            std::vector<int> differenceMatrix = HashMatrix::calculateHashValues(short_sequence[i],h_matrix.getMatrix());
            std::vector<int> row_dists = h_matrix.getBestHashRotationDists(differenceMatrix);
            costMatrix.push_back(row_dists);
        }
        m_cost_matrix = costMatrix;;
        return costMatrix;
    }

    // calculate row distances
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

        // calculate path
        auto P = calculateOptimalWarpingPath(D); // first is last match - what we want
        if (P[0].second == 0) { // 0 should(?) be error - very hacky
            m_genP = true;
        }

        return P;
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

        // copy first row of Cost matrix
        for (size_t i = 0; i < D[0].size(); i++) {
            D[0][i] = C[0][i]; // row 0 of D = C
        }

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

    //! calculate D matrix
    std::deque<std::vector<int>> calculate_accumulated_cost_matrix(std::deque<std::vector<int>> &C) {

        int N, M;
        N = C.size();
        M = C[0].size();

        std::deque<std::vector<int>> D(C.size(),std::vector<int>(C[0].size())); // accumulated cost matrix
        std::vector<int> cum_sum(C.size());
        std::vector<int> first_col;
         // cumulative sum of first column
        for  (size_t  i = 0; i < C.size(); i++) {
            first_col.push_back(C[i][0]);
        }
        std::partial_sum(first_col.begin(), first_col.end(), cum_sum.begin(), std::plus<int>());
        for (size_t i =0;  i < D.size(); i++) {
            D[i][0] = cum_sum[i];
        }


        // copy first row of Cost matrix
        for (size_t i = 0; i < D[0].size(); i++) {
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

    //! calculate path
    std::vector<std::pair<int,int> > calculateOptimalWarpingPath(std::deque<std::vector<int>> D) {

        int N = D.size(); // row size
        int n = N -1;
        int m = -1;

        int min_index = 0;
        int minVal = 1000;
        auto curr_row = D[N-1];
        for (size_t i = 0; i < curr_row.size(); i++) {  // m = D[N - 1, :].argmin()
            if (curr_row[i] < minVal) {
                minVal = curr_row[i];
                min_index = i;
            }
        }
        m = min_index; // best value of last
        std::vector<std::pair<int,int>> P; // path
        P.push_back({n,m}); // first node  add to vector

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

    public:

    //! default constructor - init with a HashMatrix
    DTHW(HashMatrix long_sequence, int roll_step) {
        this->m_long_sequence = long_sequence;
        this->m_roll_step = roll_step;
    }

    //! add to sequence - if length reached, oldest element is removed and the function returns true
    bool addToShortSequence(std::bitset<64> hashValue,size_t sequence_size) {
        bool is_limit_reached = false;
        m_sequence_limit = sequence_size;
        if (m_short_sequence.size() < sequence_size) {
            m_short_sequence.push_back(hashValue);
            is_limit_reached = false;
        } else {
            m_short_sequence.pop_front();
            m_short_sequence.push_back(hashValue);
            is_limit_reached = true;
        }
        m_current_sequence_size = m_short_sequence.size();
        return is_limit_reached;
    }

    // get best match
    int getBestMatch(HashMatrix &hmat) {
        if (m_current_sequence_size >= m_sequence_limit) {
            auto P = getBestSequence(m_short_sequence, hmat );
            int match_index =  P[0].second;  // get best match index

            return match_index;
        } else {
            std::vector<int> differenceMatrix = HashMatrix::calculateHashValues(m_short_sequence.back(),m_long_sequence.getMatrix());
            int min_col, min_row, min_value;
            HashMatrix::argmin_matrix(differenceMatrix , m_roll_step, min_col, min_row, min_value) ;
            return min_row;
        }

        return 0;

    }
};

//! checks scores based on frame distance, positional distance, and angular error
std::pair<millimeter_t,degree_t> scoring(Route &testRoute, int current_frame, Route &referenceRoute, int match_index) {
    auto currentNode = testRoute.nodes[current_frame]; // the current node in the testted route
    auto referenceNode = referenceRoute.nodes[match_index]; // matched node in the reference route - single match
    std::pair<millimeter_t, degree_t> errors;

    millimeter_t dist = RouteNode::distance(currentNode, referenceNode); // distance from the test node in mm - single match
    degree_t angDiff = RouteNode::angle_distance(currentNode, referenceNode); // angle  angular difference - single match

    errors.first = dist;
    errors.second = angDiff;

    return errors;
}

//---------------------------------- main  --------------------------------------///
int main(int argc, char **argv) {

    bool show_images = true;    // show visual
    int seq_length = 5;        // sequence length
    int roll_step = 90;         // number of rotations for a view
    cv::Size unwrapRes(180,60); // resolution of the unwrrapped video
    bool createVideo = false;    // if true, it saves unwrapped video
    bool unwrap = false;         // if true, videos will be unwrapped

    // init routes - read coordinates from csv, read and unwrap images from video
    auto route1 = Route::setup(0,roll_step, unwrap, createVideo, unwrapRes);
    auto route2 = Route::setup(1,roll_step,unwrap, createVideo, unwrapRes);

    // setup Hash matrix with route
    HashMatrix hashmat1(route1); // create a matrix with rotations
    HashMatrix hashmat2(route2); // create a matrix with rotations

    // init sequence matcher with HashMatrix
    DTHW sequence_matcher(hashmat1,roll_step); // init sequence matcher with training matrices

    // simulation
    for (size_t h = 0; h < route2.nodes.size(); h++) {

        auto hash = route2.nodes[h].image_hash; // current hash of test set
        int min_value; // best value

        // add to the memory( short sequence) - we fill the experience vector to the desired level
        sequence_matcher.addToShortSequence(hash,seq_length);

        // get the best matching frame using sequence matching
        int seq_index = sequence_matcher.getBestMatch(hashmat1);
        std::pair<int,int> match_pos = HashMatrix::getSingleMatch(hash, hashmat1, min_value, roll_step);

        auto seq_match_errors = scoring(route2, h, route1, seq_index);
        auto single_match_errors = scoring(route2, h, route1, match_pos.first);
        std::cout << "curr = " << h << " S_MATCH_err " <<  single_match_errors.first << " SEQ_MATCH_err " << seq_match_errors.first << std::endl;

        if (show_images) {
            cv::Mat conc_img1, conc_img2;
            std::vector<cv::Mat> concat_imgs({route2.nodes[h].image, route1.nodes[match_pos.first].image, route1.nodes[seq_index].image});
            cv::vconcat(concat_imgs, conc_img1);
            cv::resize(conc_img1, conc_img2, cv::Size(), 10, 10);
            cv::imshow("testing single vs seq", conc_img2);
            cv::waitKey(1);

        }

    }

    return 0;
}



