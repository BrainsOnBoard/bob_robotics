
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


class GpuDct {
    public:
    float *d_T;
    int m_size;
    int m_batch_count;
   

    


    cublasHandle_t m_handle;

    GpuDct(int n) {
        cublasCreate(&m_handle);
        cudaMalloc(&d_T, n*n*sizeof(float));  
        getDCTMatrix(n,n, d_T);   
        m_size = n;
    }

    std::bitset<64> dct(const cv::Mat &img) {
        
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

    // batch version of dct with cublas - input : list of square images
    std::vector<std::bitset<64>> batched_dct(std::vector<cv::Mat> &images) {
        m_batch_count = images.size();
        int n = m_size;
        size_t im_size = n * n;
        std::vector<std::bitset<64>> hash_batch(m_batch_count);
      
        float *d_image_array; cudaMalloc(&d_image_array, im_size * m_batch_count *sizeof(float));
        float *d_transfrom_matrix_array; cudaMalloc(&d_transfrom_matrix_array, im_size * m_batch_count * sizeof(float));
        float **d_B;   cudaMalloc((void**)&d_B, m_batch_count*sizeof(float *));
        float **d_C;   cudaMalloc((void**)&d_C, m_batch_count*sizeof(float *));
        float **d_DCT; cudaMalloc((void**)&d_DCT, m_batch_count*sizeof(float *));
        float **d_A;  cudaMalloc((void**)&d_A, m_batch_count*sizeof(float *));
        float *h_DCT = (float *) malloc(im_size*m_batch_count * sizeof(float));
        

        // read images to contigous gpu memory
        for (int i = 0; i < m_batch_count; i++) {
            cv::cuda::GpuMat gpu_mat;
            gpu_mat.upload(images[i]);
            float *d_img_data = reinterpret_cast<float*>(gpu_mat.data);
            cudaMemcpy(d_image_array + i * im_size, d_img_data, im_size*sizeof(float), cudaMemcpyDeviceToDevice);
        }
        cudaMemcpy(d_A, d_image_array, m_batch_count*sizeof(float *),cudaMemcpyDeviceToDevice);

        // read transform matrix to device array (device copy the same matrix)
        for(int i = 0 ; i < m_batch_count; i++ ) {
            cudaMemcpy(d_transfrom_matrix_array + i * im_size, d_T, im_size * sizeof(float), cudaMemcpyDeviceToDevice);    
        }
        cudaMemcpy(d_B, d_transfrom_matrix_array, m_batch_count* sizeof(float *),cudaMemcpyDeviceToDevice);

        std::cout << " matrices setup for batched gemm" << std::endl;
        auto status = batched_gpu_dct( (const float **)  d_A, (const float **) d_B,  d_C,  d_DCT, m_batch_count);
        if (status == CUBLAS_STATUS_SUCCESS) {
            std::cout << " status success" << std::endl;

            for (int i = 0; i < m_batch_count; i++) {
            
                cudaMemcpy(h_DCT + i * im_size, d_DCT[i], im_size*sizeof(float), cudaMemcpyDeviceToHost);
                cv::Mat h_mat(cv::Size(n, n), CV_32FC1, &h_DCT+ i * im_size);
                cv::Mat c_roi(h_mat, cv::Rect(0, 0, 8, 8));
                

                std::bitset<64> hash = BoBRobotics::ImgProc::DCTHash::getHashBits(c_roi);
                hash_batch.push_back(hash);
                std::cout << hash << std::endl;
            }
            

            // clean up
            cudaFree(d_A);
            cudaFree(d_B);
            cudaFree(d_C);
            cudaFree(d_DCT);
            cudaFree(d_image_array);
            cudaFree(d_transfrom_matrix_array);

            return hash_batch;
        }
    }

   

    private:

    cublasStatus_t batched_gpu_dct(const float **d_A,  const float **d_B,  float **d_C,  float **d_DCT, int batch_count) {
        // leading dimension - width of matrix
        
        const float *alpha;//1
        const float *beta; //0
        int m = m_size; // rows in d_images matrix
        int n = m;

        // T * A * T = transform matrix and the image multiplied to get dct results
        // cublas have column major ordering so have to transpose matrices (can be confusing!)
        //(BTxAT)T x B = (d_tT x d_aT)T x d_t
        cublasStatus_t stat1 = cublasSgemmBatched(   m_handle,
                                            CUBLAS_OP_T,
                                            CUBLAS_OP_N,
                                            n,  n,  n,
                                            alpha,
                                             d_B, n,
                                             d_A,  n,
                                            beta,
                                            d_C,  n,
                                            batch_count);


        cublasStatus_t stat2 = cublasSgemmBatched(  m_handle,
                                            CUBLAS_OP_T,
                                            CUBLAS_OP_N,
                                            n,  n,  n,
                                            alpha,
                                             d_C, n,
                                             d_B,  n,
                                            beta,
                                            d_DCT,  n,
                                            batch_count);

        if (stat1==CUBLAS_STATUS_SUCCESS && stat2== CUBLAS_STATUS_SUCCESS) {
            return stat1;
        } 
        return CUBLAS_STATUS_EXECUTION_FAILED;
    }
    // calculates the DCT of an image (needs to be fixed)
    std::bitset<64> gpu_dct(const float *A) {
        int n = m_size; 

        float *tmp, *DCT;
        cudaMalloc(&tmp, n*n*sizeof(float));
        cudaMalloc(&DCT,  n*n*sizeof(float));
        const float alf = 1;
        const float bet = 0;
        const float *alpha = &alf;
        const float *beta = &bet;

        // T * A * T'
        cublasSgemm(m_handle, CUBLAS_OP_T, CUBLAS_OP_N, n, n, n, alpha, d_T, n, A, n, beta, tmp, n);
        cudaDeviceSynchronize(); // N T, tmp,dt,dct
        cublasSgemm(m_handle, CUBLAS_OP_N, CUBLAS_OP_N, n, n, n, alpha, tmp, n, d_T, n, beta, DCT, n);
        cudaDeviceSynchronize();
        cudaFree(tmp);
        cv::Mat c_roi;
        cv::cuda::GpuMat g_mat(cv::Size(n, n), CV_32FC1, DCT);
        cv::cuda::GpuMat g_roi(g_mat, cv::Rect(0, 0, 8, 8));
        g_roi.download(c_roi);
        cudaFree(DCT);
        return BoBRobotics::ImgProc::DCTHash::getHashBits(c_roi);
    }

    void getDCTMatrix(int rows,int cols, float *d_Transfrom) {
        kernel_get_T<<<rows, cols>>>(d_Transfrom);
    }

};