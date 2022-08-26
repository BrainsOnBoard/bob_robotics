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
#include <bit>
#include <cstdint>



#include <thrust/device_ptr.h>
#include <thrust/extrema.h>
#include <thrust/execution_policy.h>
#include <cublas_v2.h>


const int BLOCKSIZE = 256;
const int BLOCK_DIM = 16;
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess)
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

//source:
//github.com/JonathanWatkins/CUDA/blob/master/NvidiaCourse/Exercises/transpose/transpose.cu
__global__ void kernel_transpose(float *odata, float *idata, int width, int height)
{
	__shared__ float block[BLOCK_DIM][BLOCK_DIM+1];

	// read the matrix tile into shared memory
    // load one element per thread from device memory (idata) and store it
    // in transposed order in block[][]
	unsigned int xIndex = blockIdx.x * BLOCK_DIM + threadIdx.x;
	unsigned int yIndex = blockIdx.y * BLOCK_DIM + threadIdx.y;
	if((xIndex < width) && (yIndex < height))
	{
		unsigned int index_in = yIndex * width + xIndex;
		block[threadIdx.y][threadIdx.x] = idata[index_in];
	}

    // synchronise to ensure all writes to block[][] have completed
	__syncthreads();

	// write the transposed matrix tile to global memory (odata) in linear order
	xIndex = blockIdx.y * BLOCK_DIM + threadIdx.x;
	yIndex = blockIdx.x * BLOCK_DIM + threadIdx.y;
	if((xIndex < height) && (yIndex < width))
	{
		unsigned int index_out = yIndex * height + xIndex;
		odata[index_out] = block[threadIdx.x][threadIdx.y];
	}
}

// number of blocks = number of rows * 2 - 1, number of threads/block = number of rows (<= blockSize)
/**
 * @brief ordering the distance matrix in a zig zag style for memory coalescing when calculating the
 * accumulated distance matrix.
 * Example:
 * unordered
 * [0 ,1 ,2 ,3 ,4 ]
 * [5 ,6 ,7 ,8 ,9 ]
 * [10,11,12,13,14]
 *
 * ordered:
 * [0, 5, 1, 10, 6, 2, 11, 7, 3, 12, 8, 4, 13, 9, 14]
 * using this form the cost matrix can process the diagonals in parallel as they only depend on the previous
 * 2 diagonal lines.
 * the number of blocks should be = (size of rows * 2 -1). number of threads/block = number of rows <= BLOCKSIZE
 * @param d_dist_mat the unordered cost matrix
 * @param d_ordered_dist_mat ordered cost matrix
 * @param row_n number of rows
 * @param col_n number of columns
 * @return **void
 */
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

template <typename T> // reordering the Distance matrix for displaying
/**
 * @brief reordering the distance matrix for displaying
 *
 * @param D reordered accumulated distance matrix
 * @param D_ord zig zag ordered accumulated distance matrix
 * @param D_index_matrix index matrix
 * @return **void
 */
__global__ void kernel_reorder_matrix(T *D, T *D_ord, const int *D_index_matrix) {
    int uid = blockIdx.x*blockDim.x + threadIdx.x;
    int ind = D_index_matrix[uid];
    D[ind] = D_ord[uid];
}

/**
 * @brief Calculating the accumulated cost matrix
 * @param D The accumulated cost matrix
 * @param C The cost matrix between vector1 and vector2 reordered  in zig zag style for memory coalescing
 * @param rows number of rows in the cost matrix
 * @param cols number of columns of the cost matrix
 * @return ** void
 */
__global__ void kernel_calculate_accumulated_cost_matrix(int *D, const int *C, const int rows, const int cols) {
    int bid = blockIdx.x;
    int tid = threadIdx.x;
    int uid = blockIdx.x*blockDim.x + threadIdx.x;
    extern __shared__ int s_diag[];
    // we store 3 diagonals in the shared memory - current and last 2
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
            D[c_sum + tid] = C[c_sum+tid] + min(min(up, left), up_left);
        }
        __syncthreads();
    }
}

/**
 * @brief computes a min reduce in a warp (32 threads)
 * @param s_data shared memory chunk of size 64
 * @param tid thread id
 * @return ** void
 */
__device__ void warp_reduce_min(volatile int *s_data, int tid) {
    if (tid < 32) {
        s_data[tid] = min(s_data[tid],s_data[tid + 32]);
        s_data[tid] = min(s_data[tid],s_data[tid + 16]);
        s_data[tid] = min(s_data[tid],s_data[tid +  8]);
        s_data[tid] = min(s_data[tid],s_data[tid +  4]);
        s_data[tid] = min(s_data[tid],s_data[tid +  2]);
        s_data[tid] = min(s_data[tid],s_data[tid +  1]);
    }
}

template <typename T>
/**
 * @brief adds up all elements in a warp (32 threads)
 * @param s_data shared memory chunk of size 64
 * @param tid thread id
 * @param cols number of elements (in case it is less than 64)
 * @return **void
 */
__device__ void warp_reduce_add(volatile T *s_data, int tid, int cols) {

    if (cols >= 64) s_data[tid] += s_data[tid + 32];
    if (cols >= 32) s_data[tid] += s_data[tid + 16];
    if (cols >= 16) s_data[tid] += s_data[tid +  8];
    if (cols >= 8)  s_data[tid] += s_data[tid +  4];
    if (cols >= 4)  s_data[tid] += s_data[tid +  2];
    if (cols >= 2)  s_data[tid] += s_data[tid +  1];


}

/**
 * @brief fills up a matrix with numbers [0-N]
 *
 * @param index_matrix pointer to the matrix on device
 * @return **void
 */
__global__ void kernel_fill_index_matrix(int *index_matrix) {
    int uid = blockIdx.x*blockDim.x + threadIdx.x;
    index_matrix[uid] = uid;
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

    warp_reduce_min(s_dist_mat_row, tid);
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

// roll image to the right by the number of pixels specified == turn left by x pixels
template <typename T>
__global__ void kernel_roll_image(T* rotated_image, T *image, int image_width, int pixel_to_rotate) {

    int uid = blockIdx.x*blockDim.x + threadIdx.x;
    int bid = blockIdx.x;
    int tid = threadIdx.x;

    // load each row to shred memory
    extern __shared__ T image_row[];
    image_row[tid] = image[uid];
    __syncthreads();

    if (tid <= pixel_to_rotate) {
        rotated_image[uid] =image_row[image_width - 1 - pixel_to_rotate + tid];

    } else {
        rotated_image[uid] = image_row[ -(pixel_to_rotate+1) + tid];
    }


}

// if n <= BLOCKSIZE
__global__ void kernel_get_transform_matrix(float *T) {
    int uid = blockIdx.x*blockDim.x + threadIdx.x;
    int bid = blockIdx.x;
    int tid = threadIdx.x;

    float j = (float)tid;
    float i = (float)bid;
    float N = (float)blockDim.x;
    float PI = 3.141592654;
    // different normalisation for first column

    if (bid == 0) {
        T[uid] = sqrt(1.0/N) *cos( ((2*j+1))/(2.0*N)*PI*i);
    } else {
        T[uid] = sqrt(2.0/N) *cos( ((2*j+1))/(2.0*N)*PI*i);

    }
}


__global__ void kernel_reduce(float *D, float *reduced, int N) {
    __shared__ float s_dist_mat_row[BLOCKSIZE];
    int uid = blockIdx.x*blockDim.x + threadIdx.x;
    int bid = blockIdx.x;
    int tid = threadIdx.x;

    if (uid < N) {
        s_dist_mat_row[tid] = D[uid];
        __syncthreads();

        for(unsigned int stride = (blockDim.x/2); stride > 32 ; stride /=2){
            __syncthreads();

            if(tid < stride)
            {
                s_dist_mat_row[tid]  += s_dist_mat_row[tid + stride];
            }
        }
        warp_reduce_add(s_dist_mat_row, tid, blockDim.x);
        __syncthreads();

        if (tid == 0) {
            // reduced will have a vector of values (reduced on CPU)
            reduced[bid] = s_dist_mat_row[0];
        }
    }
}

__global__ void kernel_SAD_rotations(float *M1, float *Rotations, float *temp, int num_rotations, int cols, int rows) {
    __shared__ float img_block[BLOCKSIZE];
    int tid = threadIdx.x;
    int bid = blockIdx.x;
    int uid = blockIdx.x * blockDim.x + threadIdx.x;

    // image to shared mem
    img_block[tid] = M1[uid];
    __syncthreads();

    for (int i = 0; i < num_rotations; i++) {
        // mean abs diff
        if (uid < rows*cols) {
            temp[uid+i*rows*cols] = abs(img_block[tid] - Rotations[uid+i*rows*cols])/rows*cols;
        }
    }
}

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
        // Create a handle for CUBLAS
        cublasCreate(&handle);
        gpuErrchk( cudaMalloc(&d_T, 32*32*sizeof(float)));
        gpuErrchk( cudaMalloc(&d_image, img_height*img_width*sizeof(uchar)));
        gpuErrchk( cudaMalloc(&d_rolled_image, img_height*img_width*sizeof(uchar)));
        gpuErrchk( cudaMalloc(&d_ordered_cost_mat, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_tmp_seq, d_sequence_size*sizeof(unsigned long long int)) );
        gpuErrchk( cudaMalloc(&d_cost_matrix, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_index_matrix, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_index_matrix_ord, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_accumulated_cost_mat, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_accumulated_cost_mat_ord, d_sequence_size*num_rows*sizeof(int)));
        gpuErrchk( cudaMalloc(&d_sequence, d_sequence_size*sizeof(unsigned long long int)));
        gpuErrchk( cudaMalloc(&d_hash_mat, N*sizeof(unsigned long long int)));
        gpuErrchk( cudaMemcpy(d_hash_mat, l_hash_mat, N*sizeof(unsigned long long int), cudaMemcpyHostToDevice));
        gpuErrchk( cudaMalloc(&d_rolled_images, num_rotations*img_height*img_width*sizeof(float)));

        // initialize GPU sequence matcher
        kernel_fill_index_matrix<<<num_rows, d_sequence_size>>>(d_index_matrix_ord);
        cudaDeviceSynchronize();
        kernel_order_dist_matrix<<<num_rows, d_sequence_size, BLOCKSIZE*2>>>(d_index_matrix_ord, d_index_matrix, d_sequence_size, num_rows);
        cudaDeviceSynchronize();

        //getDCTMatrix(img_height, img_width);
        std::cout << " GPU initialized" << std::endl;
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
            cv::resize(curr_img, curr_img, {w, h},2);
            curr_img.convertTo(curr_img, CV_32FC1,(1.0)/255.0);
            cudaMemcpy(d_images+i*h*w,
                       reinterpret_cast<float*>(curr_img.data),
                       (h*w)*sizeof(float),
                       cudaMemcpyHostToDevice);
        }
    }
    // gets the DCT transformation matrix (same as matlab's dctmtx(n) )
    void getDCTMatrix(int rows,int cols) {
        kernel_get_transform_matrix<<<rows, cols>>>(d_T);
    }

    // calculates the DCT of an image (needs to be fixed)
    std::bitset<64> calcDCT(float *DCT, const float *A ,const int n) {
        float *tmp;
        cudaMalloc(&tmp, n*n*sizeof(float));
        const float alf = 1;
        const float bet = 0;
        const float *alpha = &alf;
        const float *beta = &bet;

        // T * A * T'
        cublasSgemm(handle, CUBLAS_OP_T, CUBLAS_OP_N, n, n, n, alpha, d_T, n, A, n, beta, tmp, n);
        cudaDeviceSynchronize(); // N T, tmp,dt,dct
        cublasSgemm(handle, CUBLAS_OP_N, CUBLAS_OP_N, n, n, n, alpha, tmp, n, d_T, n, beta, DCT, n);
        cudaDeviceSynchronize();
        cudaFree(tmp);
        cv::Mat c_roi;
        cv::cuda::GpuMat g_mat(cv::Size(n, n), CV_32FC1, DCT);
        cv::cuda::GpuMat g_roi(g_mat, cv::Rect(0, 0, 8, 8));
        g_roi.download(c_roi);
        return DCTHash::getHashBits(c_roi);
    }


    float *getDCTpointer() {
        return d_T;
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
        gpuErrchk( cudaMemcpy(d_img, (float*)image.data, rows*cols*sizeof(float), cudaMemcpyHostToDevice) );
        for (int i = 0; i < num_rotations; i++) {
            int offset = i*cols*rows;
            kernel_roll_image<<<rows, cols, cols*sizeof(float) >>>(d_rolled_images+offset, d_img, cols,i);
        }
        cudaDeviceSynchronize();
    }

    // gets perfect memory distance matrix
    cv::Mat get_best_PM(cv::Mat &image) {
        cv::Mat img,current_image;
        cv::cvtColor(image, current_image, cv::COLOR_BGR2GRAY);
        cv::resize(current_image, current_image, {pm_width, pm_height},2);
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
        kernel_order_dist_matrix<<<num_rows, d_sequence_size, BLOCKSIZE*2>>>(d_cost_matrix, d_ordered_cost_mat, d_sequence_size, num_rows);
        cudaDeviceSynchronize();
        kernel_calculate_accumulated_cost_matrix<<<1, d_sequence_size, 3*d_sequence_size>>>(d_accumulated_cost_mat, d_ordered_cost_mat, d_sequence_size,num_rows);
        cudaDeviceSynchronize();
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

                int rot_hash_val = DCTHash::distance(element, current_hash);
                if (rot_hash_val < min) {
                    min = rot_hash_val;
                    best_rot = i;
                }
            }
        }
        std::cout << " ind " << result_offset << " rotation = " << best_rot << std::endl;
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
        cudaFree(d_hash_mat);
        cudaFree(d_sequence);
        cudaFree(d_cost_matrix);
        cudaFree(d_tmp_seq);
        cudaFree(d_ordered_cost_mat);
        cublasDestroy(handle);
    }

    private:

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
    float *d_T;
    unsigned long long int *l_best_row;
    float* d_rolled_images;  // all rotations of an image
    std::vector<uchar*> image_data_vector;
    cublasHandle_t handle;

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

