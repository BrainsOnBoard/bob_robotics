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

// from :
// https://github.com/JonathanWatkins/CUDA/blob/master/NvidiaCourse/Exercises/transpose/transpose.cu
__global__ void kernel_transpose(int *odata, int *idata, int width, int height)
{
	__shared__ int block[32][32];
	unsigned int xIndex = blockIdx.x * 32 + threadIdx.x;
	unsigned int yIndex = blockIdx.y * 32 + threadIdx.y;
	if((xIndex < width) && (yIndex < height))
	{
		unsigned int index_in = yIndex * width + xIndex;
		block[threadIdx.y][threadIdx.x] = idata[index_in];
	}
	__syncthreads();
	xIndex = blockIdx.y * 32 + threadIdx.x;
	yIndex = blockIdx.x * 32 + threadIdx.y;
	if((xIndex < height) && (yIndex < width))
	{
		unsigned int index_out = yIndex * height + xIndex;
		odata[index_out] = block[threadIdx.x][threadIdx.y];
	}
}

// number of blocks = number of rows * 2 - 1, number of threads/block = number of rows (<= blockSize)
__global__ void kernel_order_dist_matrix(int *d_dist_mat, int *d_ordered_dist_mat, int row_n, int col_n) {
    int bid = blockIdx.x;
    int tid = threadIdx.x;
    int current_row, current_col, col, row, next_row, next_col, opp_row, opp_col;
    extern __shared__ int s_diagonals[];
    int sum_diag = (row_n * (row_n + 1))/2;
    int c_sum = (bid*(bid+1))/2;

    if (bid <= row_n-1) {
        next_row = bid - tid;
        next_col = tid;
        opp_row =  row_n -1- tid;
        opp_col =  col_n -1- bid + tid;
        // load the elements from increasing and decreasing diagonals
        if ((next_row >= 0) && (next_col < bid+1)) {
            s_diagonals[tid] = d_dist_mat[next_row * col_n + next_col];
            s_diagonals[BLOCKSIZE+tid] = d_dist_mat[opp_row * col_n + opp_col];
        }
        __syncthreads();
        int diag_index = tid;
        int last = (col_n*row_n-1);
        if (diag_index <= bid) {
            d_ordered_dist_mat[c_sum+tid] = s_diagonals[diag_index];
            d_ordered_dist_mat[last - tid - c_sum] = s_diagonals[BLOCKSIZE+bid-diag_index];
        }
    } else {
        next_row = (row_n-1-tid);
        next_col = 1+((bid-row_n)+tid);
        // load the elements from the diagonals with same size
        if (next_row >= 0 && next_col < col_n && next_col > 0 && ( bid%row_n < (col_n-row_n) )) {
            s_diagonals[tid] = d_dist_mat[next_row*col_n + next_col];
        }
        __syncthreads();
        if (tid < row_n) {
            int num_elements = sum_diag  + (bid-(row_n))*row_n +tid;
            d_ordered_dist_mat[num_elements] = s_diagonals[tid];
        }
    }
}

__global__ void kernel_reorder_matrix(int *D, int *D_ord, int *D_index_matrix) {
    int bid = blockIdx.x;
    int tid = threadIdx.x;
    int uid = blockIdx.x*blockDim.x + threadIdx.x;

    int ind = D_index_matrix[uid];
    D[ind] = D_ord[uid];
}

__global__ void kernel_calculate_accumulated_cost_matrix(int *D, const int *C, const int rows, const int cols) {

    int bid = blockIdx.x;
    int tid = threadIdx.x;
    int uid = blockIdx.x*blockDim.x + threadIdx.x;
    extern __shared__ int s_diag[]; // we store 3 diagonals in the shared memory - current and last 2
    for (int i = 0; i < rows; i++) {
        int c_sum = (i*(i+1))/2;
        int row_offset = c_sum + tid;
        // increasing diagonals
        if (tid <= i && tid < rows) {
            if (tid == i) {
                // only copy element
                D[row_offset] = C[row_offset];
            }
            else if (tid == 0) {
                // first element of the column - cumulative sum
                D[c_sum] = C[c_sum] + D[c_sum - i];
            } else {
                if (tid > 0) {
                // calculating square indices
                    int c_sum_prev = ((i-1)*(i))/2;
                    int offset = (c_sum - c_sum_prev);
                    int up = D[(c_sum + tid)-offset];
                    int left = D[(c_sum + tid) - (offset +1)];
                    int up_left = D[(c_sum + tid) - (2*offset)];
                    D[c_sum + tid] = C[c_sum + tid] + min(min(up, left), up_left);
                }
            }
        }
        __syncthreads();
    }
    __syncthreads();

    // diagonal elements have the same number across iterations
    for (int i = 0; i < cols-rows; i++) {
        // next element in memory = the cum. sum of n rows + iteration * thread id
        int offset = ((rows)*(rows+1))/2 + (i*(rows) +tid);
        if (tid < rows) {
            if (tid == rows-1) {
                // only copy element (first row copy)
                D[offset] = C[offset];
            } else {
                int up = D[offset-rows+1];
                int left = D[(offset -rows)];
                int up_left = D[(offset - (2*rows-1))];
                D[offset] = C[offset] + min(min(up, left), up_left);
            }
        }
        __syncthreads();
    }

    __syncthreads();
    // start decreasing
    for (int i = 0; i <rows-1 ; i++) {
        if (tid < rows-1 -i) {
            // all elements so far
            int c_sum =cols*rows -(rows*(rows-1))/2;
            int c_sd = 0;
            for (int j = 0; j < i; j++) { c_sd += rows-1 - j; }
            c_sum+= c_sd;

            int up_offset = rows-1 - i;
            int left_offset = rows -i;
            int up_left_offset = (2*left_offset);

            int up = D[(c_sum+tid)-up_offset];
            int left = D[(c_sum+tid)-left_offset];
            int up_left = D[(c_sum+tid)-up_left_offset];
            printf("C %d, tid %d  ---- up %d, left %d, upleft %d\n", C[c_sum+tid], tid, up, left,up_left);
            D[c_sum + tid] = C[c_sum+tid] + min(min(up, left), up_left);
        }
        __syncthreads();
    }
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

        gpuErrchk( cudaMalloc(&d_ordered_cost_mat, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_tmp_seq, d_sequence_size*sizeof(unsigned long long int)) );
        gpuErrchk( cudaMalloc(&d_cost_matrix, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_sequence, d_sequence_size*sizeof(unsigned long long int)));
        gpuErrchk( cudaMalloc(&d_hash_mat, N*sizeof(unsigned long long int)));
        gpuErrchk( cudaMemcpy(d_hash_mat, l_hash_mat, N*sizeof(unsigned long long int), cudaMemcpyHostToDevice));

    }

    void testOrdering() {
        int rows = 7;
        int cols = 10;
        int *l_test = (int *) malloc(rows*cols * sizeof(int));
        int *l_ordered_test = (int *) malloc(rows*cols * sizeof(int));
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j <cols ; j++) {
                l_test[i*cols+j] = i*cols+j;
                printf("%03d, ", l_test[i*cols+j]);

            }
            std::cout << std::endl;
        }
        std::cout << std::endl << std::endl;
        int *d_test;
        int *d_ordered_test;
        int *d_index_matrix;
        int *d_D;
        gpuErrchk( cudaMalloc(&d_test, rows*cols*sizeof(int)));
        gpuErrchk( cudaMemcpy(d_test, l_test, rows*cols*sizeof(int), cudaMemcpyHostToDevice));
        gpuErrchk( cudaMalloc(&d_ordered_test, rows*cols*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_index_matrix, rows*cols*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_D, rows*cols*sizeof(int)));
        kernel_order_dist_matrix<<<cols, rows, BLOCKSIZE*2>>>(d_test, d_ordered_test, rows, cols); //
        cudaMemcpy(d_index_matrix, d_ordered_test, rows*cols*sizeof(int), cudaMemcpyDeviceToDevice);
        cudaMemcpy(l_ordered_test, d_ordered_test, rows*cols*sizeof(int), cudaMemcpyDeviceToHost);
        std::cout << std::endl << std::endl;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j <cols ; j++) {

                printf("%03d, ", l_ordered_test[i*cols+j]);
            }
            std::cout << std::endl;
        }

        kernel_calculate_accumulated_cost_matrix<<<1, rows, 3*rows>>>(d_D, d_ordered_test, rows,cols);
        cudaDeviceSynchronize();
        kernel_reorder_matrix<<<1, rows*cols>>>(d_test,d_D, d_index_matrix);
        cudaMemcpy(l_test, d_test, rows*cols*sizeof(int), cudaMemcpyDeviceToHost);
        std::cout << std::endl << std::endl;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j <cols ; j++) {

                printf("%03d, ", l_test[i*cols+j]);
            }
            std::cout << std::endl;
        }

    }

    // upload a hash sequence to the gpu
    void uploadSequence(unsigned long long int *sequence) {
        l_sequence = sequence;
        gpuErrchk( cudaMemcpy(d_sequence, sequence, d_sequence_size*sizeof(unsigned long long int), cudaMemcpyHostToDevice));
    }

    // append one element to the hash sequence and pop front
    void addToSequence(unsigned long long int *hash) {

        kernel_shift_elements<<<(d_sequence_size+255/256),d_sequence_size>>>(d_sequence,d_tmp_seq, d_sequence_size);
        cudaMemcpy(&d_sequence[d_sequence_size-1], hash, sizeof(unsigned long long int), cudaMemcpyHostToDevice);
    }

    // get distance matrix between two datasets
    void getDistanceMatrix(unsigned long long int *sequence) {
        uploadSequence(sequence);
        kernel_construct_distance_matrix<<<(d_sequence_size+255/256), d_sequence_size>>>(d_sequence, d_hash_mat, d_cost_matrix, d_sequence_size, num_rows);
    }

    // gets the current distance matrix
    void getDistanceMatrix() {
        kernel_construct_distance_matrix<<<(d_sequence_size+255/256),d_sequence_size>>>(d_sequence, d_hash_mat, d_cost_matrix, d_sequence_size, num_rows);
        cudaDeviceSynchronize();
    }

    void calculate_accumulated_cost_matrix() {
        kernel_order_dist_matrix<<<num_rows*2, d_sequence_size, num_rows>>>(d_cost_matrix, d_ordered_cost_mat, num_rows, num_cols);
    }

    // downloads the currant distance matrix in opencv mat
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
        cudaFree(d_cost_matrix);
        cudaFree(d_tmp_seq);
        cudaFree(d_ordered_cost_mat);
    }

    private:

    const int BLOCKSIZE = 256;
    unsigned long long int *l_hash_mat; // host hash mat
    unsigned long long int *d_hash_mat; // device hash mat
    unsigned long long int *l_sequence; // sequence on host
    unsigned long long int *d_sequence; // sequence on device
    unsigned long long int *d_tmp_seq;  // temporary sequence holder
    int *d_ordered_cost_mat;            // dist matrix ordered in zig zag for parallel sequence matching
    int *l_cost_matrix;                 // final cost matrix on host
    int *d_cost_matrix;                 // final cost matrix on device

    unsigned int d_sequence_size;       // size of the sequence
    int N;                              // total size of a hash matrix with rotations
    int num_rows;                       // number of unique elements in a hash matrix
    int num_cols;                       // number of rotations in the training matrix

};

