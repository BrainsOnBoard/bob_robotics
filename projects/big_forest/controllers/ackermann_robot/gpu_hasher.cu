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


const int BLOCKSIZE = 256;
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess)
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}




// number of blocks = number of rows * 2 - 1, number of threads/block = number of rows (<= blockSize)
__global__ void kernel_order_dist_matrix(int *d_dist_mat, int *d_ordered_dist_mat, int row_n, int col_n) {
    int uid = blockIdx.x*blockDim.x + threadIdx.x;
    int bid = blockIdx.x;
    int tid = threadIdx.x;
    int col, row, next_row, next_col;

    if (tid == 0) {
        d_ordered_dist_mat[0] = d_dist_mat[0];
    }
    // each block processes a diagonal line [-1,+1] direction
    // start of teh row - increasing diagonal elements
    if (bid < row_n && tid > 0 && bid > 0) {

        next_row = blockIdx.x - tid;
        next_col = tid;
        // check bounds
        if ((next_row > 0 && next_row < row_n) && (next_col > 0 && next_col < col_n)) {
            d_ordered_dist_mat[uid] = d_dist_mat[next_row*row_n + next_col];
        }

    } else {
        // diagonal line elements decreasing
        next_row = row_n - 1; // last row
        next_col = (blockIdx.x - row_n-1) + tid;       //
        // check bounds
        if ((next_row > 0 && next_row < row_n) && (next_col > 0 && next_col < col_n)) {
            d_ordered_dist_mat[uid] = d_dist_mat[next_row*row_n + next_col];
        }
    }
    __syncthreads();

}

__global__ void kernel_calculate_accumulated_cost_matrix() {

}


__device__ void warp_reduce(volatile int *s_data, int tid) {
    if (tid < 32) {
        s_data[tid] = min(s_data[tid],s_data[tid + 32]);
        s_data[tid] = min(s_data[tid],s_data[tid + 16]);
        s_data[tid] = min(s_data[tid],s_data[tid +  8]);
        s_data[tid] = min(s_data[tid],s_data[tid +  4]);
        s_data[tid] = min(s_data[tid],s_data[tid +  2]);
        s_data[tid] = min(s_data[tid],s_data[tid +  1]);
    }
}




// number of blocks = number of unique elements, number of threads/block = number of rotations (<= blockSize)
__global__ void kernel_calculateMatrixBlock_row(
                                    unsigned long long int hash,          // sequence of hash [should be the block size]
                                    unsigned long long int *d_training_matrix,      // training matrix with [blocksize] rotations for each element
                                    int *d_rotation_dist_matrix_col,                    // output matrix [reduced to best rotations]
                                    int N // number of elements
                                    ) {

    int uid = blockIdx.x*blockDim.x + threadIdx.x;
    int bid = blockIdx.x;
    int tid = threadIdx.x;
    __shared__ unsigned long long int s_rotations[BLOCKSIZE];
    __shared__ int s_dist_mat_row[BLOCKSIZE];
    __shared__ int s_dist_mat_col[BLOCKSIZE];


    s_rotations[tid] = d_training_matrix[uid]; // load each rotation row to each block of share memory
    __syncthreads();
    s_dist_mat_row[tid] = __popc(hash ^ s_rotations[tid]);
    __syncthreads();

    for(unsigned int stride = (blockDim.x/2); stride > 32 ; stride /=2){
        __syncthreads();

        if(tid < stride)
        {
            s_dist_mat_row[tid]  = min(s_dist_mat_row[tid],s_dist_mat_row[tid + stride]);

        }
    }

    warp_reduce(s_dist_mat_row, tid);
    __syncthreads();
    // save the best rotation in the column
    if (tid == 0) {
        d_rotation_dist_matrix_col[bid] = s_dist_mat_row[0];

    }

}


__global__ void kernel_construct_distance_matrix(unsigned long long int *d_sequence, unsigned long long int *d_training_matrix, int *d_distance_matrix, int sequence_size, int N) {
    int uid = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned long long int hash = d_sequence[uid];
    if (uid < sequence_size) {
        kernel_calculateMatrixBlock_row<<<N,BLOCKSIZE>>>(hash, d_training_matrix, &d_distance_matrix[uid*N], N);
    }
    cudaDeviceSynchronize();
}

__global__ void kernel_shift_elements(unsigned long long int *d_sequence, unsigned long long int *d_tmp_seq, int d_sequence_size) {
    int uid = blockIdx.x*blockDim.x + threadIdx.x;

    if (uid < d_sequence_size-1 ) {
        d_tmp_seq[uid] = d_sequence[uid+1];
        __syncthreads();
        d_sequence[uid] = d_tmp_seq[uid];
    }



}


class GPUHasher
{
    public:
    GPUHasher() {}

    // allocate memory using the training matrix [num_elements x num_rotations]
    void initGPU(unsigned long long int *l_hash_mat, int hash_mat_size, int sequence_size, int num_rotations) {

        N = hash_mat_size;
        num_rows = N/num_rotations;
        num_cols = num_rotations;
        d_sequence_size = sequence_size;


        l_cost_matrix = (int *) malloc(N * sizeof(int));
        gpuErrchk( cudaMalloc(&d_tmp_seq, d_sequence_size*sizeof(unsigned long long int)) );
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

    // append one element to the hash sequence and pop front
    void addToSequence(unsigned long long int *hash) {

        kernel_shift_elements<<<(d_sequence_size+255/256),BLOCKSIZE>>>(d_sequence,d_tmp_seq, d_sequence_size);
        cudaMemcpy(&d_sequence[BLOCKSIZE-1], hash, sizeof(unsigned long long int), cudaMemcpyHostToDevice);
    }

    // get distance matrix between two datasets
    void getDistanceMatrix(unsigned long long int *sequence) {
        uploadSequence(sequence);

        kernel_construct_distance_matrix<<<(d_sequence_size+255/256), d_sequence_size>>>(d_sequence, d_hash_mat, d_cost_matrix, d_sequence_size, num_rows);
    }

    // gets the current distance matrix
    void getDistanceMatrix() {
        //kernel_sequence_XOR<<<(N+255/256),BLOCKSIZE>>>(d_sequence, d_hash_mat, d_tmp_cost_mat, N, d_sequence_size);
        //kernel_buildCostMatrix<<<(d_sequence_size+255/256),BLOCKSIZE>>>(d_tmp_cost_mat, d_cost_matrix, N, num_rows, d_sequence_size, num_cols);
        kernel_construct_distance_matrix<<<(d_sequence_size+255/256),d_sequence_size>>>(d_sequence, d_hash_mat, d_cost_matrix, d_sequence_size, num_rows);

        cudaDeviceSynchronize();

    }

    cv::Mat downloadDistanceMatrix() {
        gpuErrchk( cudaMemcpy(l_cost_matrix, d_cost_matrix, num_rows*d_sequence_size*sizeof(int), cudaMemcpyDeviceToHost) );
        cv::cuda::GpuMat gpu_mat({d_sequence_size, num_rows, CV_32SC1, d_cost_matrix});
        cv::Mat host_mat;
        gpu_mat.download(host_mat);
        host_mat.convertTo(host_mat,CV_8UC1);

        return host_mat;
    }

    ~GPUHasher() {
        cudaFree(d_hash_mat);
        cudaFree(d_sequence);
        cudaFree(d_tmp_cost_mat);
        cudaFree(d_cost_matrix);
    }

    private:

    const int BLOCKSIZE = 256;
    unsigned long long int *l_hash_mat; // host hash mat
    unsigned long long int *d_hash_mat; // device hash mat
    unsigned long long int *l_sequence; // sequence on host
    unsigned long long int *d_sequence; // sequence on device
    unsigned long long int *d_tmp_seq;  // temporary sequence for shift elements
    int *d_tmp_cost_mat;                // sequence of cost matrices with rotations
    int *l_cost_matrix;                 // final cost matrix on host
    int *d_cost_matrix;                 // final cost matrix on device
    unsigned int d_sequence_size;       // size of the sequence
    int N;                              // total size of a hash matrix with rotations
    int num_rows;                       // number of unique elements in a hash matrix
    int num_cols;                       // number of rotations in the training matrix

};

