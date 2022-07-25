// cuda implementation of hash matching and sequence
#include "navigation/image_database.h"
#include "imgproc/dct_hash.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

// Standard C includes
#include <ctime>
#include <chrono>
#include <string.h>
#include <bitset>

#include <thrust/device_ptr.h>
#include <thrust/extrema.h>



#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess)
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

// simple hamming distance of hash against the database
__global__ void kernel_XOR( unsigned long long int hash, // the hash
                            unsigned long long int *training_matrix, // the training matrix
                            int *set_bits,     // cost matrix calculated
                            unsigned int training_matrix_size     // size of all elements in hash matrix
                            ) {
    // get thread id
    int tid = blockIdx.x*blockDim.x + threadIdx.x;
    if (tid < training_matrix_size){
        auto xor_res = hash ^ training_matrix[tid];
        set_bits[tid] =  __popc(xor_res);
    }
}

// reduce training matrix with rotations to a vector of best rotations accross the database
__global__ void kernel_reduceMat2Vector(int *d_rotation_dist_matrix,int *d_reduced_vector, int matrix_width, int number_of_elements) {
    int tid = blockIdx.x*blockDim.x + threadIdx.x;
    if (tid <number_of_elements) {
        thrust::device_ptr<int> g_ptr =  thrust::device_pointer_cast(&d_rotation_dist_matrix[tid*matrix_width]);
        int result_offset = thrust::min_element(thrust::device,g_ptr, g_ptr + matrix_width) - g_ptr;
        int min_value = *(g_ptr + result_offset);
        d_reduced_vector[tid] = min_value;
    }
}

// build matrix from sequences and best vectors
__global__ void kernel_buildCostMatrix(int *d_rotation_dist_matrix, int* d_cost_matrix, int N, int num_rows, int sequence_size, int num_rotations) {
    int tid = blockIdx.x*blockDim.x + threadIdx.x;
    if (tid < sequence_size) {
        int gridSize = (num_rows + BLOCKSIZE - 1) / BLOCKSIZE; // value determine by block size and total work
        kernel_reduceMat2Vector<<< gridSize,BLOCKSIZE>>>(&d_rotation_dist_matrix[tid*N], &d_cost_matrix[tid*num_rows], num_rotations, num_rows);
    }
}

// get hamming distance of the database against a sequence
__global__ void kernel_sequence_XOR(unsigned long long int *hash_sequence,
                                    unsigned long long int *d_training_matrix,
                                    int *d_rotation_dist_matrix,
                                    int N, // full size of matrix with rotations
                                    int sequence_size
                                    ) {
    // get thread id
    int tid = blockIdx.x*blockDim.x + threadIdx.x;
    if (tid < sequence_size) {
        kernel_XOR<<<(N+255)/256, 256>>>(hash_sequence[tid], d_training_matrix, &d_rotation_dist_matrix[tid*N], N);
    }
}



class GPUHasher
{
    public:
    GPUHasher() {}

    // allocate memory using the training matrix [num_elements x num_rotations]
    void initGPU(std::vector<std::bitset<64>> hashMat, int sequence_size, int num_rotations, int num_elements) {

        N = hashMat.size();
        num_rows = N/num_rotations;
        num_cols = num_rotations;
        d_sequence_size = sequence_size;

        gpuErrchk( cudaMalloc(&d_tmp_cost_mat, d_sequence_size*N*sizeof(int))); // this will store all the sequence matrices
        gpuErrchk( cudaMalloc(&d_cost_matrix, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_sequence, d_sequence_size*sizeof(unsigned long long int)));
        gpuErrchk( cudaMalloc(&d_hash_mat, N*sizeof(unsigned long long int)));
        gpuErrchk( cudaMemcpy(d_hash_mat, l_hash_mat, N*sizeof(unsigned long long int), cudaMemcpyHostToDevice));

    }

    // upload a hash sequence to the gpu
    void uploadSequence(unsigned long long int *sequence) {
        l_sequence = sequence;
        gpuErrchk( cudaMemcpy(d_sequence, sequence, d_sequence_size*sizeof(unsigned long long int), cudaMemcpyHostToDevice));
    }

    // get distance matrix between two datasets
    void getDistanceMatrix(unsigned long long int *sequence) {
        uploadSequence(sequence);
        kernel_sequence_XOR<<<(N+255/256),BLOCKSIZE>>>(d_sequence, d_hash_mat, d_tmp_cost_mat, N, d_sequence_size);
        kernel_buildCostMatrix<<<(d_sequence_size+255/256),BLOCKSIZE>>>(d_tmp_cost_mat, d_cost_matrix, N, num_rows, d_sequence_size, num_cols);
    }

    cv::Mat downloadDistanceMatrix(bool show) {
        gpuErrchk( cudaMemcpy(l_cost_matrix, d_cost_matrix, num_rows*d_sequence_size*sizeof(int), cudaMemcpyDeviceToHost) );
        cv::cuda::GpuMat gpu_mat({d_sequence_size, num_rows, CV_32SC1, d_cost_matrix});
        cv::Mat host_mat;
        gpu_mat.download(host_mat);
        host_mat.convertTo(host_mat,CV_8UC1);

        if (show) {
            cv::normalize(host_mat, host_mat, 0, 255, cv::NORM_MINMAX);
            cv::applyColorMap(host_mat, host_mat, cv::COLORMAP_JET);
            cv::imshow("gpu_mat", host_mat);
            cv::waitKey(1);
        }

        return host_mat;
    }

    private:

    const int BLOCKSIZE = 256;
    unsigned long long int *l_hash_mat; // host hash mat
    unsigned long long int *d_hash_mat; // device hash mat
    unsigned long long int *l_sequence; // sequence on host
    unsigned long long int *d_sequence; // sequence on device
    int *d_tmp_cost_mat;                // sequence of cost matrices with rotations
    int *l_cost_matrix;                 // final cost matrix on host
    int *d_cost_matrix;                 // final cost matrix on device
    unsigned int d_sequence_size;       // size of the sequence
    int N;                              // total size of a hash matrix with rotations
    int num_rows;                       // number of unique elements in a hash matrix
    int num_cols;                       // number of rotations in the training matrix

};

