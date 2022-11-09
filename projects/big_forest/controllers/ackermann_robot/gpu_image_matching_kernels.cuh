// gpu image matching functions
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

__global__ void kernel_get_distmat_column(unsigned long long int* d_training_matrix, unsigned long long int *column) {
    int uid = blockIdx.x*blockDim.x + threadIdx.x;
    column[uid] = d_training_matrix[uid*BLOCKSIZE];
}

__global__ void kernel_get_distmat_from_rotations(unsigned long long int *rotations, unsigned long long int *d_training_matrix_column, int *rot_dist_mat) {
    // for all rotations
    int uid = blockIdx.x*blockDim.x + threadIdx.x;
    for (int i = 0; i < BLOCKSIZE; i++) {
        rot_dist_mat[uid+ i*BLOCKSIZE] = __popcll(rotations[i] ^ d_training_matrix_column[uid]);
    }

}

__global__ void kernel_simple_dist_mat(unsigned long long int *training_dataset, unsigned long long int *test_dataset, int *distance_matrix, int train_size, int test_size) {
    int uid = blockIdx.x*blockDim.x + threadIdx.x;
    for (int i = 0; i < test_size; i++) {
        if (uid < train_size) {
            distance_matrix[i*train_size + uid] = __popcll(test_dataset[i] ^ training_dataset[uid]);
        }
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
    s_dist_mat_row[tid] = __popcll(hash ^ s_rotations[tid]);
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



template <typename T>
__global__ void kernel_reduce(T *D, T *reduced, int N) {
    __shared__ T s_dist_mat_row[BLOCKSIZE];
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

/**
this kernel calculates a single match of a testing image against the training database
**/
__global__ void kernel_SAD_single_matches(float *training_images, float *testing_image, float *temp, int rows, int cols, int N_training) {
    __shared__ float img_block[BLOCKSIZE];
    int tid = threadIdx.x;
    int bid = blockIdx.x;
    int uid = blockIdx.x * blockDim.x + threadIdx.x;

    // image to shared mem
    img_block[tid] = testing_image[uid % (rows*cols)];
    __syncthreads();
    if (uid < N_training * rows*cols) {
        temp[uid] = abs(img_block[tid] - training_images[uid]);
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
