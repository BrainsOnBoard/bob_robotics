// scoring the datasets
// Dataset_evaluator.cpp

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "imgproc/roll.h"
#include <string>
#include "common/path.h"
#include "common/stopwatch.h"
#include "common/pose.h"
#include "common/bn055_imu.h"
#include "common/background_exception_catcher.h"
#include "navigation/image_database.h"
#include "imgproc/dct_hash.h"

// Standard C includes
#include <ctime>
#include <chrono>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>


#include <plog/Log.h>
#include <plog/Init.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Appenders/DebugOutputAppender.h>
#include "plog/Initializers/RollingFileInitializer.h"

#include <thrust/device_ptr.h>
#include <thrust/extrema.h>

#include "../route_setup.cuh"

#include "../gpu_hasher.cu"
#include "imgproc/gpu_dct.h"

constexpr int RESIZED_WIDTH = 256;
constexpr int RESIZED_HEIGHT = 256;

class DatasetEvaluator {
    private:

    // -------------------- variables for NORDLEND and ALDERLEY datasets ----------------//
    std::vector<cv::Mat> m_training_images;
    std::vector<cv::Mat> m_test_images;
    //-----------------------------------------------------------------------------------//

    //--------------------- variables for panoramic datasets-----------------------------//
    int m_roll_step;
    int m_skipstep;
    Route m_training_route;
    Route m_testing_route;
    //-----------------------------------------------------------------------------------//

    std::string m_databases_folder_path;
    std::string m_dataset_type; // NORDLAND, ALDERLEY, FOREST, WEBOTS
    std::string m_training_images_path;
    std::string m_test_images_path;
    bool m_isPanoramic; // if not panoramic, turning angle will not be scored
    cv::Size m_img_res; // resolution of the dataset images
    std::string m_difference_method; // hash, pixel, sequence

    std::string dataset_name; // name of the dataset
    std::string nordland_training_season;
    std::string nordland_testing_season;
    int norland_section;

    std::vector<std::bitset<64>> m_training_hashes;
    std::vector<std::bitset<64>> m_test_hashes;

    HashMatrix m_hashMat; // only for hash matching
    GPUHasher g_hasher;

    public:


    std::string fillZeros(int n_zero, std::string old_string) const {

        std::stringstream ss;
        ss << std::string(n_zero - old_string.length(), '0') << old_string;
        std::string new_string = ss.str();
        return new_string;
    }

    std::string make_string_path(std::string const& path1, int img_number) {
        std::stringstream ss;
        ss << path1 << "Image"<< fillZeros(5, std::to_string(img_number)) << ".jpg";

        return ss.str();
    }

    std::string make_string_path_NORDLAND(std::string const& path1, int img_number) {
        std::stringstream ss;
        ss << path1 << "images-"<< fillZeros(5, std::to_string(img_number)) << ".png";

        return ss.str();
    }

    std::vector<std::bitset<64>> read_NORDLAND(std::string const& season, std::vector<cv::Mat> &images, int section = 1, cv::Size size = {64,64}) {

        std::vector<std::bitset<64>> hashes;

        int section1_start = 0;
        int section1_end = 1149;

        int section2_start = 1150;
        int section2_end = 2299;

        int section3_start = 2300;
        int section3_end = 3449;

        int dataset_start;
        int dataset_end;

        if     (section == 1) { dataset_start = section1_start; dataset_end = section1_end;}
        else if(section == 2) { dataset_start = section2_start; dataset_end = section2_end;}
        else if(section == 3) { dataset_start = section3_start; dataset_end = section3_end;}
        else {
            std::cout << " invalid option" << std::endl;
            exit(0);
        }

        //std::string nordland_path = "64x32-grayscale-1fps/";
        //std::string full_path = m_databases_folder_path + nordland_path + season + "/";

        std::string nordland_path = "Partitioned_Nordland_Dataset_lowres/Dataset_images/test/" + season + "_images_test/" + "section" + std::to_string(section) + "/";
        std::string full_path = m_databases_folder_path + nordland_path;

        GpuDct gdct(size.width);
        std::vector<cv::Mat> square_images;

        for (int i = dataset_start; i < dataset_end; i++) {
            //std::string image_path = make_string_path_NORDLAND(full_path, i);

            std::string image_path = full_path + std::to_string(i) + ".png";

            //if (i % 100 ==0 ) std::cout << image_path << std::endl;
            //std::cout << image_path << std::endl;
            cv::Mat img = cv::imread(image_path ,0);
            cv::Mat output;
            cv::Mat square_img;
            cv::equalizeHist(img, output);
            cv::resize(output, output, size);
            images.push_back(output);
            cv::resize(img, square_img, {size.width, size.width});
            square_img.convertTo(square_img, CV_32F, 1.0/255);
            square_images.push_back(square_img);
            auto hash = gdct.dct(square_img);
            hashes.push_back(hash);
        }

        return hashes;
    }

    std::vector<std::bitset<64>> read_ALDERLEY(std::string ID, std::vector<cv::Mat> &images, int dataset_start, int dataset_end) {

        std::vector<std::bitset<64>> hashes;
        GpuDct gdct(64);

        // read FRAMESA or FRAMESB
        std::string frames_a = "ALDERLEY_TEST/alderley/FRAMESA/";
        std::string frames_b = "ALDERLEY_TEST/alderley/FRAMESB/";

        std::string file_path;
        if (ID == "A") {
            file_path = m_databases_folder_path + frames_a;
        }
        else if (ID == "B") {
            file_path = m_databases_folder_path + frames_b;
        } else {
            std::cout << "wrong id" <<  std::endl;
        }

        for (int i = dataset_start; i < dataset_end; i++) {
            if (i % 100 == 0) std::cout << "i = " << i << std::endl;
            std::string image_path = make_string_path(file_path, i);
            cv::Mat img = cv::imread(image_path ,0);
            cv::Mat output;
            cv::resize(img, output, {64,32});
            //cv::cvtColor(output, output, cv::COLOR_RGB2GRAY);
            images.push_back(output);
            output.convertTo(img, CV_32F, 1.0/255);
            auto hash = gdct.dct(output);
            hashes.push_back(hash);
        }

        return hashes;
    }





    // DATASET TYPES = {WEBOTS, FOREST, NORDLAND, ALDERLEY}
    DatasetEvaluator(std::string databases_folder_path, std::string training_image_path, std::string test_image_path, cv::Size resolution, int skipstep, std::string difference_method, bool isPanoramic, std::string dataset_type = "A", std::string norland_season_train = "SUMMER", std::string norland_season_test = "FALL", bool isSequence = false, int section = 1) {
        m_isPanoramic = isPanoramic;
        m_img_res = resolution;
        m_skipstep = skipstep;
        m_test_images_path = test_image_path;
        m_training_images_path = training_image_path;
        m_difference_method = difference_method;
        m_databases_folder_path = databases_folder_path;

        dataset_name = dataset_type;
        nordland_training_season = norland_season_train;
        nordland_testing_season = norland_season_test;
        norland_section = section;


        read_dataset(training_image_path, resolution, true, dataset_type, norland_season_train, section);// read training dataset
        read_dataset(test_image_path, resolution, false, dataset_type, norland_season_test, section);   // read testing dataset
        int N_training = m_training_images.size();
        int N_testing =  m_test_images.size();
        if (!isPanoramic) {
            g_hasher.init_GPU_for_single_match(m_training_images, m_test_images, m_training_images.size(), m_test_images.size(), resolution, isSequence);
        }
    }
    /**
     * method = hash - DCT HASH, sequence - sequence hashes, pixel - pixel matching (perfect memory[PM])
    */


    void read_dataset(std::string dataset_name, cv::Size image_resolution, bool isTrainingDataset, std::string dataset_type, std::string nordland_season = "summer", int section = 1) {

        std::vector<int> csv_columns_webots = {9, 0 , 1 , 2 , 3 , 4, 5, 7, 8, 6}; // simulation dataset ordering
        std::vector<int> csv_columns_forest = {0, 12, 13, 14, 16, 5, 6, 7, 8, 9}; // real world dataset ordering

        std::vector<cv::Mat> train_imgs;
        std::vector<cv::Mat> test_imgs;


        if (m_isPanoramic) {
            // do the rotation - rotation hash matrix version

            if (isTrainingDataset) {
                if (dataset_type == "WEBOTS") {
                    Route route = Route(dataset_name,256, m_skipstep, false, false, m_img_res, false, false, csv_columns_webots);
                    std::cout << "creating hash matrix: " << std::endl;
                    HashMatrix hashMat(route.nodes, 256);
                    m_hashMat = hashMat;
                    m_training_route = route;
                }
                else if (dataset_type == "FOREST") {
                    Route route = Route(dataset_name,256, m_skipstep, false, false, m_img_res, false, false, csv_columns_forest);
                    HashMatrix hashMat(route.nodes, 256);
                    m_hashMat = hashMat;
                    m_training_route = route;
                } else {
                    std::cout << "unknown dataset type ... exiting" << std::endl;
                    exit(0);
                }

                ///------gpu init------
            } else {
                if (dataset_type == "WEBOTS") {
                    Route route = Route(dataset_name,256, m_skipstep, false, false, m_img_res,false,false, csv_columns_webots);
                    m_testing_route = route;
                    for (int i = 0; i < route.nodes.size(); i++) {
                        cv::Mat curr_img = route.nodes[i].image;
                        cv::resize(curr_img, curr_img, {256,256});
                        curr_img.convertTo(curr_img, CV_32F, 1.0 / 255);
                        m_test_images.push_back(curr_img);
                    }
                }
                else if (dataset_type == "FOREST") {
                    Route route = Route(dataset_name,256, m_skipstep, false, false, m_img_res, false,false,csv_columns_forest);
                    m_testing_route = route;
                    for (int i = 0; i < route.nodes.size(); i++) {
                    //    m_test_images.push_back(curr_img);
                    }
                }
            }

        // ALDERLEY and NORLAND dataset only needs to be read without CSV file for filenames
        } else {

            if (dataset_type == "ALDERLEY") {
                int start = 1;
                int end_A = 16960;
                int end_B = 14607;

                if (isTrainingDataset) {
                    m_training_hashes = read_ALDERLEY("A", train_imgs, start, end_A);
                    m_training_images = train_imgs;
                } else {
                    m_test_hashes = read_ALDERLEY("B", test_imgs, start, end_B);
                    m_test_images = test_imgs;
                }

            }

            else if (dataset_type == "NORDLAND") {
                if (isTrainingDataset) {
                    m_training_hashes= read_NORDLAND(nordland_season, train_imgs, section, image_resolution);
                    m_training_images = train_imgs;
                } else {
                    m_test_hashes = read_NORDLAND(nordland_season, test_imgs, section, image_resolution);
                    m_test_images = test_imgs;
                }

            } else {
                std::cout << "unknown dataset type ... exiting" << std::endl;
            }
        }
    }

    void score_dataset() {
        if (dataset_name == "NORDLAND") {
            std::vector<std::pair<int,int>> scores_PM;
            std::vector<std::pair<int,int>> scores_hash;
            std::vector<int> score_diff_PM;
            std::vector<int> score_diff_hash;
            int total_score_PM = 0;
            int total_score_hash = 0;
            // pixel matching
            cv::Mat dist_mat_all = g_hasher.get_best_PM_single_match(scores_PM);

            // hash matching
            cv::Mat hash_dist_mat;
            int *d_hash_dist_mat = g_hasher.get_single_hash_difference_matrix(scores_hash, hash_dist_mat);


            for (int i =0; i < scores_PM.size(); i++) {
                int diff = abs(scores_PM[i].first - scores_PM[i].second);
                score_diff_PM.push_back( diff );
                if (diff < 10) {
                    total_score_PM++;
                }
            }

            for (int i =0; i < scores_hash.size(); i++) {
                int diff = abs(scores_hash[i].first - scores_hash[i].second);
                score_diff_hash.push_back( diff );
                if (diff < 10) {
                    total_score_hash++;
                }
            }

            std::string d_name = dataset_name + nordland_training_season + nordland_testing_season + std::string("section") + std::to_string(norland_section);
            std::cout << d_name <<  " score [hash] = " << total_score_hash << " score [PM] = " << total_score_PM << std::endl;
            std::ofstream myFile(d_name);

            // Send the column name to the stream
            myFile << "hash" <<"," << "pixel" << "\n";
            // Send data to the stream
            for(int i = 0; i < score_diff_hash.size(); ++i)
            {
                myFile << score_diff_hash.at(i) << "," << score_diff_PM.at(i) << "\n";
            }
            // Close the file
            myFile.close();
        } else {
            // init gpu -------------
            int d_sequence_size = 128;
            int roll_step = 256;
            int hash_mat_size = m_hashMat.getMatrix().size();
            auto hm = m_hashMat.getMatrix();
            unsigned long long int *l_hash_mat = (unsigned long long int*) malloc(hash_mat_size * sizeof(unsigned long long int));
            unsigned long long int *l_sequence = (unsigned long long int*) malloc(d_sequence_size * sizeof(unsigned long long int));
            m_hashMat.getHashMatUL(l_hash_mat);
            std::vector<cv::Mat> training_images;
            for (int i = 0; i < m_training_route.nodes.size(); i++) { training_images.push_back(m_training_route.nodes[i].image);}

            g_hasher.initGPU(l_hash_mat, hash_mat_size, d_sequence_size, 256, 256, 256);
            //g_hasher.upload_database(training_images, 256, 256);


            //for (int s = d_sequence_size; s < hash_mat_size/256; s++) {
            for (int s = 0; s < m_test_images.size(); s++) {

                g_hasher.addToSequence(m_test_images[s]);
                g_hasher.getDistanceMatrix();
                std::cout << " seq :" << s <<  std::endl;

                cv::Mat host_mat1 = g_hasher.downloadDistanceMatrix();
                cv::normalize(host_mat1, host_mat1, 0, 255, cv::NORM_MINMAX);
                host_mat1.convertTo(host_mat1,CV_8UC1);
                cv::applyColorMap(host_mat1, host_mat1, cv::COLORMAP_JET);


                g_hasher.calculate_accumulated_cost_matrix();
                std::pair<int,int> min_idx = g_hasher.getMinIndex(hm[s*roll_step],hm);
                cv::Mat host_mat2 = g_hasher.downloadAccumulatedCostMatrix();
                cv::normalize(host_mat2, host_mat2, 0, 255, cv::NORM_MINMAX);
                host_mat2.convertTo(host_mat2,CV_8UC1);
                cv::applyColorMap(host_mat2, host_mat2, cv::COLORMAP_JET);

                cv::Mat combined;
                cv::vconcat(host_mat1, host_mat2, combined);
                cv::imshow("gpu_mat2", combined);
                cv::waitKey(100);
            }
        }





    }

    void get_closest_node () {

    }

    void get_angle_error() {

    }

    void score_place_recognition() {
        // check how close the match is - by using either the difference of frame (NORDLAND, ALDERLEY) - or by mm by using own datasets

        // Alderley - file is provided to score matches
        // Nordland - frame numbers are the same for train and test
        // own dataset - score can be given by calculating the distane of the recorded coordinates
    }

    void generate_score_csv(std::string dataset_name ) {}

    void score_angular_differences() {}

    void generate_score_matrix() {

    }





};
