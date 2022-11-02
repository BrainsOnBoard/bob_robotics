
#pragma once
#include <cublas_v2.h>
#include "dct_hash.h"


// if n <= BLOCKSIZE
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

__device__ void device_bitarray(unsigned long long int *addr, int *bit_array, int tid) {
    // n |= 1UL << i;

    if (bit_array[tid] > 0) {
        unsigned long long int n = 1ULL << tid;
        atomicAdd(addr, n);
    }
}


// threads per block = 64 x n_images, blocks = n_images
__global__ void kernel_compute_hash(float *dct_imgs, unsigned long long int *hash, int n_images, int cols) {
    int bid = blockIdx.x;
    int tid = threadIdx.x;

    int roi_offset =tid % 8 + ((tid / 8) * cols) ;
    __shared__ float s_array[64];
    __shared__ float array[64];
    s_array[threadIdx.x] = dct_imgs[roi_offset + bid * blockIdx.x];
    array[threadIdx.x]   = dct_imgs[roi_offset + bid * blockIdx.x];
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
    array[tid] = (array[tid] > median);
    __syncthreads();
    // each block computes a hash
    device_bitarray(hash + bid, (int*)array, tid);

}



class GpuDct {
    public:
    float *d_T;
    int m_size;
    int m_batch_count;
    cublasHandle_t m_handle;

    ~GpuDct() {
        cudaFree(d_T);
        cublasDestroy(m_handle);
    }

    GpuDct(int n) {
        cublasCreate(&m_handle);
        cudaMalloc(&d_T, n*n*sizeof(float));
        getDCTMatrix(n,n, d_T);
        m_size = n;
    }

    std::bitset<64> dct(cv::Mat &img) {

        cv::cuda::GpuMat g_mat;
        g_mat.upload(img);
        const float *img_ptr = reinterpret_cast<float*> (g_mat.data);
        auto hash = gpu_dct(img_ptr);

        return hash;
    }

    std::bitset<64> dct(float *img) {
        std::bitset<64> hash = gpu_dct(img);
        return hash;
    }

    std::vector<std::bitset<64>> batch_dct(std::vector<cv::Mat> &h_images) {
        int n = m_size;
        int im_size = n*n;
        int n_images = h_images.size();

        float *d_Arr; cudaMalloc(&d_Arr, n_images*im_size*sizeof(float));

        for (int i = 0; i < n_images; i++ ) {
            cv::Mat image = h_images[i];
            cudaMemcpy(d_Arr + i * im_size, reinterpret_cast<float*>(image.data), im_size, cudaMemcpyHostToDevice);
        }

        std::vector<std::bitset<64>> hashes = gpu_stream_dct(d_Arr, n_images , n,0);
        cudaFree(d_Arr);
        return hashes;
    }


    std::vector<std::bitset<64>> gpu_stream_dct(const float *d_images, float n_images, int n, cudaStream_t s) {
        int im_size = n*n;
        const float alf = 1.0;
        const float bet = 0.0;
        const float *alpha = &alf;
        const float *beta = &bet;

        float *d_DCT_mats; cudaMallocAsync(&d_DCT_mats, im_size*n_images*sizeof(float), s);
        float *h_DCT_mats; cudaMallocHost(&h_DCT_mats ,im_size*n_images*sizeof(float));

        for (int i = 0; i < n_images; i++ ) {
            float *h_DCT; cudaMallocHost(&h_DCT,im_size*sizeof(float));
            float *tmp; cudaMallocAsync(&tmp, n*n*sizeof(float),s);
            float *d_DCT; cudaMallocAsync(&d_DCT, im_size*sizeof(float),s);

            cublasSetStream(m_handle,s);
            cublasSgemm(m_handle, CUBLAS_OP_T, CUBLAS_OP_N, n, n, n, alpha, d_T, n, d_images + i * n *n, n, beta, tmp, n);
            cudaStreamSynchronize(s);
            cublasSetStream(m_handle,s);
            cublasSgemm(m_handle, CUBLAS_OP_N, CUBLAS_OP_N, n, n, n, alpha, tmp, n, d_T, n, beta, d_DCT, n);
            cudaStreamSynchronize(s);
            cudaMemcpyAsync(d_DCT_mats + i * im_size, d_DCT, im_size, cudaMemcpyDeviceToDevice,s);

            cudaFreeHost(h_DCT);
            cudaFree(tmp);
            cudaFree(d_DCT);
        }

        cudaStreamSynchronize(s);
        std::vector<std::bitset<64>> hashes;

        unsigned long long int *dhash; cudaMallocAsync(&dhash, n_images*sizeof(unsigned long long int),s);
        unsigned long long int *lhash; cudaMallocHost(&lhash, n_images*sizeof(unsigned long long int));
        kernel_compute_hash<<<n_images, 64, 64, s>>>(d_DCT_mats, dhash, n_images,n);
        cudaMemcpyAsync(lhash, dhash, n_images*sizeof(unsigned long long int), cudaMemcpyDeviceToHost,s);
        cudaStreamSynchronize(s);
        for (int i = 0; i < n_images; i++) { std::bitset<64> hash(lhash[i]); hashes.push_back(hash);}

        cudaFreeHost(h_DCT_mats);
        cudaFree(d_DCT_mats);
        cudaFree(dhash);
        cudaFreeHost(lhash);
        return hashes;

    }

    private:


    // calculates the DCT of an image (needs to be fixed)
    std::bitset<64> gpu_dct(const float *A) {
        int n = m_size;

        float *tmp, *DCT;
        cudaMalloc(&tmp, n*n*sizeof(float));
        cudaMalloc(&DCT,  n*n*sizeof(float));
        const float alf = 1.0;
        const float bet = 0.0;
        const float *alpha = &alf;
        const float *beta = &bet;

        // T * A * T'
        cublasSgemm(m_handle, CUBLAS_OP_T, CUBLAS_OP_N, n, n, n, alpha, d_T, n, A, n, beta, tmp, n);
        cudaDeviceSynchronize();
        cublasSgemm(m_handle, CUBLAS_OP_N, CUBLAS_OP_N, n, n, n, alpha, tmp, n, d_T, n, beta, DCT, n);
        cudaDeviceSynchronize();

        unsigned long long int *dhash;
        unsigned long long int lhash[1];
        cudaMalloc(&dhash, 1*sizeof(unsigned long long int));
        kernel_compute_hash<<<1, 64>>>(DCT, dhash, 1,n);
        cudaMemcpy(lhash, dhash, sizeof(unsigned long long int), cudaMemcpyDeviceToHost);
        std::bitset<64> barray(lhash[0]);
        cudaFree(DCT);
        cudaFree(tmp);
        cudaFree(dhash);

        return barray;
    }

    void getDCTMatrix(int rows,int cols, float *d_Transfrom) {
        kernel_get_T<<<rows, cols>>>(d_Transfrom);
    }

};
