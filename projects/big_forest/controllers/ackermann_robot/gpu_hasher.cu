#pragma once
// cuda implementation of hash matching and sequence
#include "gpu_image_matching_kernels.cuh"
#include "navigation/image_database.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "imgproc/roll.h"
#include "imgproc/gpu_dct.h"
// Standard C includes
#include <ctime>
#include <chrono>
#include <string.h>
#include <bitset>
#include <bit>
#include <cstdint>
#include <thrust/device_ptr.h>
#include <thrust/extrema.h>
#include <thrust/execution_policy.h>
#include <cublas_v2.h>


class GPUHasher
{
    public:
    GPUHasher() {}

    // allocate memory using the training matrix [num_elements x num_rotations]
    void initGPU(unsigned long long int *l_hash_mat,
                int hash_mat_size,
                int sequence_size,
                int num_rotations,
                int img_width,
                int img_height
                ) {

        N = hash_mat_size;
        num_rows = N/num_rotations;
        num_cols = num_rotations;
        d_sequence_size = sequence_size;
        l_cost_matrix = (int *) malloc(N * sizeof(int));
        l_best_row = (unsigned long long int *) malloc(num_rotations * sizeof(unsigned long long int));
        l_sequence = (unsigned long long int *) malloc(d_sequence_size * sizeof(unsigned long long int));
        l_rot_img_data = (uchar *) malloc(img_height*img_width * sizeof(uchar));
        l_accumulated_cost_matrix = (int *) malloc(d_sequence_size * num_rows * sizeof(int));
        m_image_width = img_width;
        m_image_height = img_height;

        gpu_dct = new GpuDct(img_width);


        gpuErrchk( cudaMalloc(&d_image, img_height*img_width*sizeof(uchar)));
        gpuErrchk( cudaMalloc(&d_rolled_image, img_height*img_width*sizeof(uchar)));
        gpuErrchk( cudaMalloc(&d_ordered_cost_mat, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_tmp_seq, d_sequence_size*sizeof(unsigned long long int)) );

        // for rot dist mat
        gpuErrchk( cudaMalloc(&training_route, N/num_rotations*sizeof(unsigned long long int)) );
        gpuErrchk( cudaMalloc(&current_rotations, num_rotations*sizeof(unsigned long long int)) );
        gpuErrchk( cudaMalloc(&rot_dist_mat, num_rotations*(N/num_rotations)*sizeof(int)) );


        gpuErrchk( cudaMalloc(&d_cost_matrix, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_index_matrix, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_index_matrix_ord, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_accumulated_cost_mat, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_accumulated_cost_mat_ord, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_sequence, d_sequence_size*sizeof(unsigned long long int)));
        gpuErrchk( cudaMalloc(&d_hash_mat, N*sizeof(unsigned long long int)));
        gpuErrchk( cudaMemcpy(d_hash_mat, l_hash_mat, N*sizeof(unsigned long long int), cudaMemcpyHostToDevice));
        gpuErrchk( cudaMalloc(&d_rolled_images, num_rotations*img_height*img_width*sizeof(float)));

        unsigned long long int *l_sequence = (unsigned long long int *) malloc(d_sequence_size * sizeof(unsigned long long int));
        for (int i = 0; i < d_sequence_size; i++) {
            l_sequence[i] = l_hash_mat[i*num_rotations];
        }
        uploadSequence(l_sequence);

        // initialize GPU sequence matcher
        kernel_fill_index_matrix<<<num_rows, d_sequence_size>>>(d_index_matrix_ord);
        cudaDeviceSynchronize();
        kernel_order_dist_matrix<<<num_rows, d_sequence_size, BLOCKSIZE*2>>>(d_index_matrix_ord, d_index_matrix, d_sequence_size, num_rows);
        cudaDeviceSynchronize();
        kernel_get_distmat_column<<<((N/num_rotations)+255)/256,256>>>(d_hash_mat, training_route);


        //getDCTMatrix(img_height, img_width);
        std::cout << " GPU initialized" << std::endl;
    }

    void init_GPU_for_single_match(std::vector<cv::Mat> &training_imgs,
                                  std::vector<cv::Mat> &test_images,
                                  int N_training,
                                  int N_testing,
                                  cv::Size size,
                                  bool isSequence,
                                  int sequence_size = 128) {
        d_sequence_size = sequence_size;
        unsigned long long int l_training_hashes[N_training];
        unsigned long long int l_testing_hashes[N_testing];
        img_rows = size.height;
        img_cols = size.width;
        this->N_training = N_training;
        this->N_testing = N_testing;
        num_rows = N_training;
        GpuDct gdct(size.width);

        gpuErrchk( cudaMalloc(&d_training_hashes, N_training*sizeof(unsigned long long int)));
        gpuErrchk( cudaMalloc(&d_testing_hashes, N_testing*sizeof(unsigned long long int)));
        gpuErrchk( cudaMalloc(&d_training_images, N_training*size.height*size.width*sizeof(float)));
        gpuErrchk( cudaMalloc(&d_testing_images, N_testing*size.height*size.width*sizeof(float)));
        gpuErrchk( cudaMalloc(&d_temp, size.height*size.width*N_training*sizeof(float)));
        gpuErrchk( cudaMalloc(&d_reduced_blocks, N_training*size.height*sizeof(float)));
        gpuErrchk( cudaMalloc(&d_dist_mat_PM, N_training*N_testing*sizeof(float)));
        gpuErrchk( cudaMalloc(&d_single_distance_matrix, N_testing*N_training*sizeof(int)));

        // for sequence
        gpuErrchk( cudaMalloc(&d_cost_matrix, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_index_matrix, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_index_matrix_ord, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_accumulated_cost_mat, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_accumulated_cost_mat_ord, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_sequence, d_sequence_size*sizeof(unsigned long long int)));

        // initialize GPU sequence matcher
        kernel_fill_index_matrix<<<num_rows, d_sequence_size>>>(d_index_matrix_ord);
        cudaDeviceSynchronize();
        kernel_order_dist_matrix<<<num_rows, d_sequence_size, BLOCKSIZE*2>>>(d_index_matrix_ord, d_index_matrix, d_sequence_size, num_rows);
        cudaDeviceSynchronize();


        for (int i = 0; i < training_imgs.size(); i++) {
            cv::Mat curr_img = training_imgs[i];
            //cv::equalizeHist(curr_img, curr_img);
            cv::resize(curr_img, curr_img, size,cv::INTER_CUBIC);
            curr_img.convertTo(curr_img, CV_32FC1,(1.0)/255.0);
            gpuErrchk(cudaMemcpy(d_training_images+(i*size.width*size.height),
                    reinterpret_cast<float*>(curr_img.data),
                    (size.height*size.width)*sizeof(float),
                    cudaMemcpyHostToDevice));

            auto hash = gdct.dct(d_training_images+(i*size.width*size.height));
            unsigned long long int hash_ull = hash.to_ullong();
            l_training_hashes[i] = hash_ull;
        }

        for (int i = 0; i < test_images.size(); i++) {
            cv::Mat curr_img = test_images[i];
            //cv::equalizeHist(curr_img, curr_img);
            cv::resize(curr_img, curr_img, size,cv::INTER_CUBIC);
            curr_img.convertTo(curr_img, CV_32FC1,(1.0)/255.0);
            gpuErrchk(cudaMemcpy(d_testing_images+i*size.height*size.width,
                    reinterpret_cast<float*>(curr_img.data),
                    (size.height*size.width)*sizeof(float),
                    cudaMemcpyHostToDevice));

            auto hash = gdct.dct(d_testing_images+i*size.height*size.width);
            unsigned long long int hash_ull = hash.to_ullong();
            l_testing_hashes[i] = hash_ull;
        }

        gpuErrchk( cudaMemcpy(d_training_hashes, l_training_hashes, N_training*sizeof(unsigned long long int), cudaMemcpyHostToDevice));
        gpuErrchk( cudaMemcpy(d_testing_hashes, l_testing_hashes, N_testing*sizeof(unsigned long long int), cudaMemcpyHostToDevice));

        std::cout << " gpu for singles matches initialized " << N_testing << std::endl;
    }



    void upload_database(std::vector<cv::Mat> images, int w, int h) {

        pm_width = w;
        pm_height = h;
        N_images = images.size();
        int num_rotations = w; // rotations in the image = width of image
        num_blocks = (h*w+BLOCKSIZE-1)/BLOCKSIZE;
        cudaMalloc(&d_temp, h*w*num_rotations*sizeof(float));
        cudaMalloc(&d_reduced_blocks, num_rotations*num_blocks*sizeof(float));
        cudaMalloc(&d_SSD_rotations, num_rotations*sizeof(float));
        cudaMalloc(&d_dist_mat_PM, N_images*num_rotations*sizeof(float));
        cudaMalloc(&d_images, (h*w)*N_images*sizeof(float));

        for (int i = 0; i < images.size(); i++) {
            cv::Mat curr_img = images[i];
            cv::cvtColor(curr_img, curr_img, cv::COLOR_BGR2GRAY);
            cv::resize(curr_img, curr_img, {w, h},cv::INTER_CUBIC);
            curr_img.convertTo(curr_img, CV_32FC1,(1.0)/255.0);
            cudaMemcpy(d_images+i*h*w,
                       reinterpret_cast<float*>(curr_img.data),
                       (h*w)*sizeof(float),
                       cudaMemcpyHostToDevice);
        }
    }

    // print a matrix on the GPU
    template<typename T>
    void printMatrix(T *d_matrix,int rows, int cols, int rowLim, int colLim) {
        T tmp[rows*cols];
        cudaMemcpy(tmp, d_matrix, rows*cols*sizeof(T), cudaMemcpyDeviceToHost);
        std::cout << std::fixed;
        std::cout << std::setprecision(4);
        std::cout << "-------------------------------------" << std::endl;
        std::cout << "-------------Matrix start------------" << std::endl;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (i < rowLim && j < colLim) {
                    std::cout << "["<< tmp[i*cols+j] << "]";
                } else {
                    break;
                }

            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout << "-------------Matrix end--------------" << std::endl;
        std::cout << "-------------------------------------" << std::endl;
    }

    // rolls images to get all the rotations - return rolled images on host
    void get_rotations(const cv::Mat &image, const int num_rotations) {
        auto img_data = image.data;
        int cols = image.cols;
        int rows = image.rows;
        auto type = image.type();
        float *d_img;
        gpuErrchk( cudaMalloc(&d_img, rows*cols*sizeof(float)) );
        gpuErrchk( cudaMemcpy(d_img, reinterpret_cast<float*>(image.data), rows*cols*sizeof(float), cudaMemcpyHostToDevice) );
        for (int i = 0; i < num_rotations; i++) {
            int offset = i*cols*rows;
            kernel_roll_image<<<rows, cols, cols*sizeof(float) >>>(d_rolled_images+offset, d_img, cols,i);
        }
        cudaDeviceSynchronize();
    }



    static float* stream_rotations(const cv::Mat &image, std::vector<cv::Mat> &rotated_images, const int num_rotations, bool download_host=false) {

        int cols = image.cols;
        int rows = image.rows;
        float *d_i;        gpuErrchk( cudaMalloc(&d_i, rows*cols*sizeof(float)) );
        float *d_rolled_ims; gpuErrchk( cudaMalloc(&d_rolled_ims, cols*rows*num_rotations*sizeof(float)));
        gpuErrchk( cudaMemcpy(d_i, reinterpret_cast<float*>(image.data), rows*cols*sizeof(float), cudaMemcpyHostToDevice) );
        float *h_images; cudaMallocHost(&h_images, num_rotations * rows*cols*sizeof(float));


        // create streams
        cudaStream_t *streams = (cudaStream_t *) malloc(num_rotations * sizeof(cudaStream_t));
        for (int i =0; i < num_rotations; i++) { cudaStreamCreate(&(streams[i])); }
        // roll images parallel

        if (download_host) {
            for (int i = 0; i < num_rotations; i++) {
                kernel_roll_image<<<rows, cols, cols*sizeof(float), streams[i] >>>(d_rolled_ims+i*rows*cols, d_i, cols,i);
                cudaMemcpyAsync(h_images + i * rows * cols, d_rolled_ims + i * rows * cols, rows*cols*sizeof(float), cudaMemcpyDeviceToHost,streams[i]);
            }


            // destroy stream and wait until all complete
            for (int i =0; i < num_rotations; i++) { cudaStreamDestroy(streams[i]); }
            if (streams) free(streams);
            // copy back all the rotations to cpu memory
            for (int i = 0; i < num_rotations; i++ ) {
                cv::Mat rolled(rows,cols, CV_8UC1,h_images + i * rows * cols);
                rolled.convertTo(rolled, CV_32F, 1.0 / 255);
                rotated_images.push_back(rolled);
            }
            if (d_rolled_ims) cudaFree(d_rolled_ims);

        } else {
            for (int i = 0; i < num_rotations; i++) {
                kernel_roll_image<<<rows, cols, cols*sizeof(float), streams[i] >>>(d_rolled_ims+i*rows*cols, d_i, cols,i);
            }
            // destroy stream and wait until all complete
            for (int i =0; i < num_rotations; i++) { cudaStreamDestroy(streams[i]); }
            if (streams) free(streams);
        }

        // cleanup

        if (d_i) cudaFree(d_i);
        if (h_images) cudaFreeHost(h_images);
        return d_rolled_ims;
    }


     // rolls images to get all the rotations - return rolled images on host
    static void get_rotations(const cv::Mat &image, std::vector<cv::Mat> &rolled_images, const int num_rotations) {
        auto img_data = image.data;
        int cols = image.cols;
        int rows = image.rows;
        float *d_img;
        float *d_rolled_ims;
        cudaMalloc(&d_rolled_ims, cols*rows*num_rotations*sizeof(float));
        gpuErrchk( cudaMalloc(&d_img, rows*cols*sizeof(float)) );
        gpuErrchk( cudaMemcpy(d_img, reinterpret_cast<float*>(image.data), rows*cols*sizeof(float), cudaMemcpyHostToDevice) );
        for (int i = 0; i < num_rotations; i++) {
            int offset = i*cols*rows;
            kernel_roll_image<<<rows, cols, cols*sizeof(float) >>>(d_rolled_ims+offset, d_img, cols,i);
            cv::Mat img;
            float tmp_img[rows*cols];
            cudaMemcpy(tmp_img, d_rolled_ims+offset, rows*cols*sizeof(float), cudaMemcpyDeviceToHost);
            img.data = reinterpret_cast<uchar*>(tmp_img);
            rolled_images.push_back(img);

        }
        cudaDeviceSynchronize();
        cudaFree(d_rolled_ims);
    }



    int * get_single_hash_difference_matrix(std::vector<std::pair<int,int>> &scores, cv::Mat &distance_matrix) {
        int threads = 256;
        int blocks = int(N_training+threads-1/threads);
        kernel_simple_dist_mat<<<blocks,threads>>>(d_training_hashes, d_testing_hashes, d_single_distance_matrix, N_training, N_testing);


        cv::cuda::GpuMat gpu_mat({N_training, N_testing, CV_32SC1, d_single_distance_matrix});
        cv::Mat temp_mat;
        gpu_mat.download(temp_mat);

        cv::normalize(temp_mat, temp_mat, 0, 255, cv::NORM_MINMAX);
        temp_mat.convertTo(temp_mat, CV_8UC1);
        cv::applyColorMap(temp_mat, temp_mat, cv::COLORMAP_JET);

        scores = get_closest_matches_from_dist_mat( d_single_distance_matrix);
        distance_matrix = temp_mat;
        return d_single_distance_matrix;
    }

    template <typename T>
    std::vector<std::pair<int,int>> get_closest_matches_from_dist_mat(T *dist_mat) {
        std::vector<std::pair<int,int>> scores;
        for (int i = 0; i < N_testing; i++) {
            std::pair<int, int> pair;

            thrust::device_ptr<T> g_ptr =  thrust::device_pointer_cast(&dist_mat[i*N_training]);
            int result_offset = thrust::min_element( g_ptr, g_ptr + (N_training) ) -g_ptr;
            int min_score = *(g_ptr + result_offset);
            pair.first = i;
            pair.second = result_offset;
            scores.push_back(pair);
        }
        return scores;

    }

    cv::Mat get_best_PM_single_match(std::vector<std::pair<int,int>> &scores) {

        int N_blocks = N_training * img_rows;
        //std::cout << "N train = " << N_training << " img rows = " << img_rows << " N blocks = "<< N_blocks << std::endl;
        for (int i = 0; i < N_testing; i++) {
           // if ((i % 100) == 0) { std::cout << "i= " << i << std::endl;}
            // absolute differencing which gives a difference matrix for all images
            kernel_SAD_single_matches<<<N_blocks,BLOCKSIZE>>>
                                            (d_training_images,
                                            d_testing_images+i*img_rows*img_cols,
                                            d_temp,
                                            img_rows,
                                            img_cols,
                                            N_training);

            cudaDeviceSynchronize();
            // summing up the values in the difference matrix
            for (int j = 0; j < N_training; j++) {
                kernel_reduce<<<img_rows,BLOCKSIZE>>>(d_temp+j*img_rows*img_cols, d_reduced_blocks+j*img_rows, img_rows*img_cols);
            }

            cudaDeviceSynchronize();
            kernel_reduce<<<N_training,img_rows>>>(d_reduced_blocks, d_dist_mat_PM+i*N_training, N_training*img_rows);
            cudaDeviceSynchronize();
        }
        cudaDeviceSynchronize();

        float* temp_array = (float *) malloc(N_training*N_testing * sizeof(float));
        cudaMemcpy(temp_array, d_dist_mat_PM, N_training*N_testing*sizeof(float), cudaMemcpyDeviceToHost);
        cv::Mat temp_mat(cv::Size(N_training,N_testing), CV_32FC1, temp_array);
        cv::normalize(temp_mat, temp_mat, 0, 255, cv::NORM_MINMAX);
        temp_mat.convertTo(temp_mat, CV_8UC1);
        cv::applyColorMap(temp_mat, temp_mat, cv::COLORMAP_JET);

        scores = get_closest_matches_from_dist_mat(d_dist_mat_PM);

        return temp_mat;

    }

    // gets perfect memory distance matrix with rotations
    cv::Mat get_best_PM(cv::Mat &image) {
        cv::Mat img,current_image;
        cv::cvtColor(image, current_image, cv::COLOR_BGR2GRAY);
        cv::resize(current_image, current_image, {pm_width, pm_height},cv::INTER_CUBIC);
        current_image.convertTo(img, CV_32FC1,(1.0)/255.0);
        int rows = pm_height;//img.rows;
        int cols = pm_width;//img.cols;
        int num_rotations = pm_width;
        get_rotations(img,num_rotations); // get all rotations of current image
        for (int i = 0; i < N_images;i++) {
            kernel_SAD_rotations<<<num_blocks,BLOCKSIZE>>>
                                         (d_images+i*rows*cols,
                                         d_rolled_images,
                                         d_temp,
                                         num_rotations,
                                         cols,
                                         rows);
            cudaDeviceSynchronize();
            for (int j = 0; j < num_rotations; j++) {
                kernel_reduce<<<num_blocks,BLOCKSIZE>>>(d_temp+j*rows*cols, d_reduced_blocks+j*num_blocks, rows*cols);
            }

            cudaDeviceSynchronize();
            kernel_reduce<<<num_rotations,num_blocks>>>(d_reduced_blocks, d_dist_mat_PM+i*num_rotations, num_blocks*num_rotations);
            cudaDeviceSynchronize();

        }

        kernel_roll_image<<<N_images, num_rotations, num_rotations*sizeof(float) >>>(d_dist_mat_PM, d_dist_mat_PM, num_rotations,num_rotations/2);
        float temp_array[N_images*num_rotations];
        cudaMemcpy(temp_array, d_dist_mat_PM, N_images*num_rotations*sizeof(float), cudaMemcpyDeviceToHost);
        cv::Mat temp_mat(cv::Size(num_rotations, N_images), CV_32FC1, temp_array);
        cv::normalize(temp_mat, temp_mat, 0, 255, cv::NORM_MINMAX);
        temp_mat.convertTo(temp_mat, CV_8UC1);
        cv::applyColorMap(temp_mat, temp_mat, cv::COLORMAP_JET);
        return temp_mat;
    }

    // upload a hash sequence to the gpu*
    void uploadSequence(unsigned long long int *sequence) {
        l_sequence = sequence;
        gpuErrchk( cudaMemcpy(d_sequence, l_sequence, d_sequence_size*sizeof(unsigned long long int), cudaMemcpyHostToDevice));
    }

    // append one element to the hash sequence and pop front
    void addToSequence(unsigned long long int *hash) {

        kernel_shift_elements<<<(d_sequence_size+255/256),d_sequence_size>>>(d_sequence,d_tmp_seq, d_sequence_size);
        cudaMemcpy(&d_sequence[d_sequence_size-1], hash, sizeof(unsigned long long int), cudaMemcpyHostToDevice);
    }

    void addToSequence(cv::Mat &image) {
        auto hash = gpu_dct->dct(image);
        unsigned long long int ulhash[1];
        ulhash[0] = hash.to_ullong();
        kernel_shift_elements<<<(d_sequence_size+255/256),d_sequence_size>>>(d_sequence,d_tmp_seq, d_sequence_size);
        cudaMemcpy(&d_sequence[d_sequence_size-1], ulhash, sizeof(unsigned long long int), cudaMemcpyHostToDevice);
    }

    // get distance matrix between two datasets
    void getDistanceMatrix(unsigned long long int *sequence, bool with_rotations = true) {
        uploadSequence(sequence);
        if (with_rotations) {
            kernel_construct_distance_matrix<<<(d_sequence_size+255/256), d_sequence_size>>>(d_sequence, d_hash_mat, d_cost_matrix, d_sequence_size, num_rows);
        } else {
            int threads = d_sequence_size;
            int blocks = int(N_training+threads-1/threads);
            kernel_simple_dist_mat<<<blocks,d_sequence_size>>>(d_training_hashes, d_sequence, d_cost_matrix, N_training, d_sequence_size);
        }
    }

    // gets the current distance matrix
    void getDistanceMatrix(bool with_rotations = true) {
        if (with_rotations) {
            kernel_construct_distance_matrix<<<(d_sequence_size+255/256),d_sequence_size>>>(d_sequence, d_hash_mat, d_cost_matrix, d_sequence_size, num_rows);
            cudaDeviceSynchronize();
        } else {
            int threads = d_sequence_size;
            int blocks = int(N_training+threads-1/threads);
            kernel_simple_dist_mat<<<blocks,d_sequence_size>>>(d_training_hashes, d_sequence, d_cost_matrix, N_training, d_sequence_size);
        }
    }

    // upload a vector of images to the GPU
    static void upload_hash_database(std::vector<cv::Mat> images, unsigned long long int *hashes_d_ptr) {
        GpuDct gpu_dct(images[0].size().width);
        std::vector<std::bitset<64>> hashes;
        for (int i = 0; i < images.size(); i++) {
            auto hash = gpu_dct.dct(images[i]);
            hashes.push_back(hash);
        }
        upload_hash_database(hashes, hashes_d_ptr);
    }

    // upload hashes to the gpu
    static void upload_hash_database(std::vector<std::bitset<64>> hashes, unsigned long long int *hashes_d_ptr) {
        int num_hash = hashes.size();
        unsigned long long int ull_hashes[num_hash];
        cudaMalloc(&hashes_d_ptr, num_hash*sizeof(unsigned long long int));
        for (int i = 0; i < num_hash; i++) {
            ull_hashes[i] = hashes[i].to_ullong();
        }
        cudaMemcpy(hashes_d_ptr, ull_hashes, num_hash*sizeof(unsigned long long int), cudaMemcpyHostToDevice);
    }


    void calculate_accumulated_cost_matrix() {
        kernel_order_dist_matrix<<<num_rows, d_sequence_size, BLOCKSIZE*2>>>(d_cost_matrix, d_ordered_cost_mat, d_sequence_size, num_rows);
        cudaDeviceSynchronize();
        kernel_calculate_accumulated_cost_matrix<<<1, d_sequence_size, 3*d_sequence_size>>>(d_accumulated_cost_mat, d_ordered_cost_mat, d_sequence_size,num_rows);
        cudaDeviceSynchronize();
        // reordering the matrix (so it's human readable)
        kernel_reorder_matrix<<<num_rows, d_sequence_size>>>(d_accumulated_cost_mat_ord,d_accumulated_cost_mat, d_index_matrix);
    }

    // calculates the place and the rotation of a hash from a hash matrix
    std::pair<int,int> getMinIndex(std::bitset<64> current_hash, std::vector<std::bitset<64>> hashmat) {
        thrust::device_ptr<int> g_ptr =  thrust::device_pointer_cast(&d_accumulated_cost_mat_ord[(d_sequence_size-1)*num_rows]);
        int result_offset = thrust::min_element( g_ptr, g_ptr + (num_rows-1) ) -g_ptr;
        int min_value = *(g_ptr + result_offset);
        int min = 10000;
        int best_rot = -1;
        for (int i = 0; i < num_cols; i++) {
            if (result_offset >= 0 && result_offset < num_rows) {
                std::bitset<64> element = hashmat[result_offset*num_cols+i];

                int rot_hash_val = BoBRobotics::ImgProc::DCTHash::distance(element, current_hash);
                if (rot_hash_val < min) {
                    min = rot_hash_val;
                    best_rot = i;
                }
            }
        }
        return {result_offset, best_rot};
    }

    // downloads the accumulated cost matrix in a cv::Mat object
    cv::Mat downloadAccumulatedCostMatrix() {

        gpuErrchk( cudaMemcpy(l_accumulated_cost_matrix, d_accumulated_cost_mat_ord, num_rows*d_sequence_size*sizeof(int), cudaMemcpyDeviceToHost) );
        cv::cuda::GpuMat gpu_mat({d_sequence_size, num_rows, CV_32SC1, d_accumulated_cost_mat_ord});
        cv::Mat host_mat;
        gpu_mat.download(host_mat);

        return host_mat;
    }

    // downloads the currant distance matrix in opencv mat
    cv::Mat downloadDistanceMatrix() {
        gpuErrchk( cudaMemcpy(l_cost_matrix, d_cost_matrix, num_rows*d_sequence_size*sizeof(int), cudaMemcpyDeviceToHost) );
        cv::cuda::GpuMat gpu_mat({d_sequence_size, num_rows, CV_32SC1, d_cost_matrix});
        cv::Mat host_mat;
        gpu_mat.download(host_mat);
        return host_mat;
    }

    ~GPUHasher() {

        if (d_ordered_cost_mat) cudaFree(d_ordered_cost_mat);
       // if (l_cost_matrix) free(l_cost_matrix);
      //  if (l_accumulated_cost_matrix) free(l_accumulated_cost_matrix);
        if (d_cost_matrix) cudaFree(d_cost_matrix);
        if (d_index_matrix) cudaFree(d_index_matrix);
        if (d_index_matrix_ord) cudaFree(d_index_matrix_ord);
        if (d_accumulated_cost_mat) cudaFree(d_accumulated_cost_mat);
        if (d_accumulated_cost_mat_ord) cudaFree(d_accumulated_cost_mat_ord);
        if (d_image) cudaFree(d_image);
        if (d_temp) cudaFree(d_temp);
        if (d_reduced_blocks) cudaFree(d_reduced_blocks);
        if (d_rolled_images) cudaFree(d_rolled_images);
        if (d_SSD_rotations) cudaFree(d_SSD_rotations);
        if (d_dist_mat_PM) cudaFree(d_dist_mat_PM);
        if (d_images) cudaFree(d_images);
        if (d_hash_mat) cudaFree(d_hash_mat);
        if (d_sequence) cudaFree(d_sequence);
        if (d_tmp_seq) cudaFree(d_tmp_seq);
     //   if (l_hash_mat) free(l_hash_mat);
     //   if (l_sequence) free(l_sequence);
        if (d_training_images) cudaFree(d_training_images);
        if (d_testing_images) cudaFree(d_testing_images);
        if (d_training_hashes) cudaFree(d_training_hashes);
        if (d_testing_hashes) cudaFree(d_testing_hashes);
        if (d_single_distance_matrix) cudaFree(d_single_distance_matrix);


    }

    private:

    std::vector<float *> f_pointers;
    std::vector<unsigned long long int*> ull_pointers;
    std::vector<int*> i_pointers;
    GpuDct *gpu_dct;

    unsigned long long int *l_hash_mat; // host hash mat
    unsigned long long int *d_hash_mat; // device hash mat
    unsigned long long int *l_sequence; // sequence on host
    unsigned long long int *d_sequence; // sequence on device
    unsigned long long int *d_tmp_seq;  // temporary sequence holder
    int *d_ordered_cost_mat;            // dist matrix ordered in zig zag for parallel sequence matching
    int *l_cost_matrix;                 // final cost matrix on host
    int *l_accumulated_cost_matrix;
    int *d_cost_matrix;                 // final cost matrix on device
    int *d_index_matrix;                // hold the indices for the ordered cost matrix
    int *d_index_matrix_ord;
    int *d_accumulated_cost_mat;        // accumulated cost matrix
    int *d_accumulated_cost_mat_ord;    // accumulated cost matrix

    unsigned long long int *current_rotations;
    unsigned long long int *training_route;
    int *rot_dist_mat;

    unsigned long long int *l_best_row;
    float* d_rolled_images;  // all rotations of an image
    std::vector<uchar*> image_data_vector;


    // perfect memory-----
    float *d_temp;
    int num_blocks;
    float *d_reduced_blocks;
    float *d_SSD_rotations;
    float *d_dist_mat_PM;
    float *d_images; // image database
    int pm_width;
    int pm_height;
    int N_images;
    //----------------------

    //--- for single matches --
    float *d_training_images;
    float *d_testing_images;
    unsigned long long int *d_training_hashes;
    unsigned long long int *d_testing_hashes;
    int *d_single_distance_matrix;
    int N_training;
    int N_testing;
    int img_rows; // height of image
    int img_cols; // width of image

    unsigned int d_sequence_size;       // size of the sequence
    int N;                              // total size of a hash matrix with rotations
    int num_rows;                       // number of unique elements in a hash matrix
    int num_cols;                       // number of rotations in the training matrix
    uchar *d_image;
    uchar *d_rolled_image;
    uchar *l_rot_img_data;
    int m_image_width;
    int m_image_height;

};

