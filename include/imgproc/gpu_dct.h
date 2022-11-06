
#pragma once
#include <cublas_v2.h>


__global__ void kernel_get_T(float *T) {
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



// threads per block = 64 x n_images, blocks = n_images
__global__ void kernel_compute_hash(float *dct_imgs, int *bit_array, int cols) {
    int bid = blockIdx.x;
    int tid = threadIdx.x;

    int roi_offset =tid % 8 + ((tid / 8) * cols) ;
    __shared__ float s_array[64];
    __shared__ float sb_array[64];
    s_array[threadIdx.x] = dct_imgs[roi_offset + bid * blockIdx.x];
    sb_array[threadIdx.x] = dct_imgs[roi_offset + bid * blockIdx.x];
    __syncthreads();


    // sort arrays
    for (int i = 0; i < cols/2; i++)
    {
        int j = tid;
        if (j % 2 == 0 && j<cols-1) {
            if (s_array[j+1] < s_array[j]) {
                float temp = s_array[j];
                s_array[j] = s_array[j+1];
                s_array[j+1] = temp;
            }
        }
        __syncthreads();
        if (j % 2 == 1 && j<cols-1) {
            if (s_array[j+1] < s_array[j]) {
                float temp = s_array[j];
                s_array[j] = s_array[j+1];
                s_array[j+1] = temp;
            }
        }
        __syncthreads();
    }
    __syncthreads();

    const float median = (s_array[31] + s_array[32]) /2;
    if (sb_array[tid] > median) { bit_array[tid] = 1; }
    else { bit_array[tid] = 0; }
    __syncthreads();
}


class GpuDct {
    public:
    float *d_T;
    int m_size;
    cublasHandle_t m_handle;

    GpuDct(int n) {
        cublasCreate(&m_handle);
        cudaMalloc(&d_T, n*n*sizeof(float));
        getDCTMatrix(n,n, d_T);
        m_size = n;
    }

    std::bitset<64> dct(float *d_img, cudaStream_t s = 0) {
        std::bitset<64> hash = gpu_dct(d_img, s);
        return hash;
    }

    std::bitset<64> dct(const cv::Mat &img) {

        cv::cuda::GpuMat g_mat;
        g_mat.upload(img);
        const float *img_ptr = reinterpret_cast<float*> (g_mat.data);
        auto hash = gpu_dct(img_ptr);
        return hash;
    }


    std::bitset<64> dct(const cv::Mat &img, cudaStream_t s) {
        int n = img.size().width;
        float *img_ptr; cudaMallocHost(&img_ptr, n*n*sizeof(float));
        float *g_mat; cudaMallocAsync(&g_mat, n*n*sizeof(float),s);
        img_ptr = reinterpret_cast<float*>(img.data);
        cudaMemcpyAsync(g_mat, img_ptr, n*n * sizeof(float), cudaMemcpyHostToDevice, s);
        auto hash = gpu_dct(img_ptr, s);
        cudaStreamSynchronize(s);
        cudaFree(g_mat);
        cudaFreeHost(img_ptr);
        return hash;
    }





    private:

    // calculates the DCT of an image (needs to be fixed)
    std::bitset<64> gpu_dct(const float *A, cudaStream_t s = 0) {
        int n = m_size;
        const float alf = 1;
        const float bet = 0;
        const float *alpha = &alf;
        const float *beta = &bet;
        float *tmp, *DCT;
        cudaMallocAsync(&tmp, n*n*sizeof(float),s);
        cudaMallocAsync(&DCT,  n*n*sizeof(float),s);
        // T * A * T'
        cublasSetStream(m_handle,s);
        cublasSgemm(m_handle, CUBLAS_OP_T, CUBLAS_OP_N, n, n, n, alpha, d_T, n, A, n, beta, tmp, n);
        cudaStreamSynchronize(s);
        cublasSgemm(m_handle, CUBLAS_OP_N, CUBLAS_OP_N, n, n, n, alpha, tmp, n, d_T, n, beta, DCT, n);
        cudaStreamSynchronize(s);

        int *dbit_array; cudaMallocAsync(&dbit_array, 64*sizeof(int),s);
        int *hasharr; cudaMallocHost(&hasharr, 64*sizeof(int));
        kernel_compute_hash<<<1, 64, 0, s>>>(DCT, dbit_array,n);
        cudaMemcpyAsync(hasharr, dbit_array, 64*sizeof(int), cudaMemcpyDeviceToHost,s);
        cudaStreamSynchronize(s);

        std::bitset<64> binary;
        for (size_t i = 0; i < 64; i++) binary.set(i, hasharr[i]);
        cudaFree(tmp);
        cudaFree(DCT);
        cudaFreeHost(hasharr);
        return binary;
    }

    void getDCTMatrix(int rows,int cols, float *d_Transfrom) {
        kernel_get_T<<<rows, cols>>>(d_Transfrom);
    }
};
