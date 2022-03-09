#include "tracer.h"

// little bit of GPU testing biatch

namespace CudaTracing {
    __global__ void test_kernel() {
        printf("Hello from the GPU\n");
    }

    __global__ void update(std::vector<Eigen::Vector3f> *rays, int N, Pose *start_pose, loop_closure::LoopClosureConfig *config) {
        
        //printf("Inside the kernel");
        // first: calculate the index and stride size for this operation
        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;

        for(int i = index; i < N; i+= stride) {
            
        }
    }

    int helloWorld() {
        test_kernel<<<1,1>>>();
        cudaDeviceSynchronize();
        return 0;
    }

    /*
    * Function, which takes in the rays, calls upon the kernel function until ready
    */
    void updateRays(std::vector<Eigen::Vector3f> *rays, Pose *start_pose, loop_closure::LoopClosureConfig *config) {
        //printf("Inside the function running the kernel");
        // Run kernel on 1M elements on the GPU
        int blockSize = 256;
        int numBlocks = (rays->size() + blockSize - 1) / blockSize;
        update<<<numBlocks, blockSize>>>(rays, rays->size(), start_pose, config);
    }
}

// Kernel function to add the elements of two arrays
/*__global__
void add(int n, float *x, float *y)
{
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  for (int i = index; i < n; i += stride)
    y[i] = x[i] + y[i];
}*/



/*int main(void)
{
    int N = 1 << 20;
    float *x, *y;

    // Allocate Unified Memory – accessible from CPU or GPU
    cudaMallocManaged(&x, N * sizeof(float));
    cudaMallocManaged(&y, N * sizeof(float));

    // initialize x and y arrays on the host
    for (int i = 0; i < N; i++)
    {
        x[i] = 1.0f;
        y[i] = 2.0f;
    }

    // Run kernel on 1M elements on the GPU
    int blockSize = 256;
    int numBlocks = (N + blockSize - 1) / blockSize;

    std::cout << "num blocks: " << numBlocks << " Bloc-size: " << blockSize << std::endl;
    add<<<numBlocks, blockSize>>>(N, x, y);

    // Wait for GPU to finish before accessing on host
    cudaDeviceSynchronize();

    // Check for errors (all values should be 3.0f)
    float maxError = 0.0f;
    for (int i = 0; i < N; i++)
        maxError = fmax(maxError, fabs(y[i] - 3.0f));
    std::cout << "Max error: " << maxError << std::endl;

    // Free memory
    cudaFree(x);
    cudaFree(y);
    
    return 0;
}*/