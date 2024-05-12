#include <hip/hip_runtime.h>
#include <hip/hip_runtime_api.h>
#include <iostream>
#include <vector>

using namespace std;

constexpr auto BLOCK_SIZE = 256;
typedef unsigned long long DataType;

void check_hip_error(hipError_t &err)
{
  if (err != hipSuccess)
  {
    std::cerr << "Error: " << hipGetErrorString(err) << std::endl;
    exit(err);
  }
}

// step 1
__global__ void reduce_baseline(DataType *in, DataType *out, int len)
{
  __shared__ DataType sdata[BLOCK_SIZE];
  int tid = threadIdx.x;
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  sdata[tid] = in[i];
  __syncthreads();

  for (int s = 1; s < blockDim.x; s *= 2)
  {
    if (tid % (2 * s) == 0)
    {
      sdata[tid] += sdata[tid + s];
    }
    // s=1: 0-1,2-3,4-5....
    // s=2: 0(0,1)-2(2,3),4(4,5)-6(6,7)...
    // s=4: 0(0-3)-4(4-7),8(8-11)-12(12-15)
    __syncthreads();
  }

  if (tid == 0)
  {
    atomicAdd(out, sdata[0]);
  }
}

// step 2
__global__ void reduce_avoid_divergent_branching(DataType *in, DataType *out,
                                                 int len)
{
  // TODO
  // 避免同一个warp存在不同的执行路径
  __shared__ DataType sdata[BLOCK_SIZE];
  int tid = threadIdx.x;
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  sdata[tid] = in[i];
  __syncthreads();

  for (int s = 1; s < blockDim.x; s *= 2)
  {
    int index = s * tid << 1;
    if (index < blockDim.x)
    {
      sdata[index] += sdata[index + s];
    }
    // s=1: 0-1,2-3,4-5...
    // s=2: 0-2,4-6,8-10...
    // s=4: 0-4,8-12...
    __syncthreads();
  }

  if (tid == 0)
  {
    atomicAdd(out, sdata[0]);
  }
}

// step 3
__global__ void reduce_avoid_bank_conflicts(DataType *in, DataType *out,
                                            int len)
{
  // TODO
  // 同一个 warp 内的不同线程试图访问同一 bank 内的不同数据时发生冲突
  // 64 threads per wavefront and 32 bank of LDS
  __shared__ DataType sdata[BLOCK_SIZE];
  int tid = threadIdx.x;
  int i = blockIdx.x * blockDim.x + threadIdx.x;

  sdata[tid] = in[i];
  __syncthreads();

  for (int s = blockDim.x / 2; s > 0; s = s / 2)
  {
    if (tid < s)
    {
      sdata[tid] += sdata[tid + s];
    }
    // s=128: 0-128,1-129.... (+0,+128)
    // s=64: 0(0,128)-64(64,192),1(1,129)-65(65,193) (+0,+128,+64,+64+128)
    // s=32: 0(0,64,128,192)-32(32,96,160,224)...  (0,32,64,96,128,160,192,224)
    __syncthreads();
  }

  if (tid == 0)
  {
    atomicAdd(out, sdata[0]);
  }
}

// step 4
__global__ void reduce_first_add_during_load(DataType *in, DataType *out,
                                             int len)
{
  // TODO
  __shared__ DataType sdata[BLOCK_SIZE];
  int tid = threadIdx.x;
  int i = blockIdx.x * blockDim.x * 2 + threadIdx.x;
  sdata[tid] = in[i] + in[i + blockDim.x];
  __syncthreads();

  for (int s = blockDim.x / 2; s > 0; s >>= 1)
  {
    if (tid < s)
    {
      sdata[tid] += sdata[tid + s];
    }
    __syncthreads();
  }

  if (tid == 0)
  {
    atomicAdd(out, sdata[0]);
  }
}

__device__ void warpReduce1(volatile DataType *sdata, int tid)
{
  sdata[tid] += sdata[tid + 64];
  sdata[tid] += sdata[tid + 32];
  sdata[tid] += sdata[tid + 16];
  sdata[tid] += sdata[tid + 8];
  sdata[tid] += sdata[tid + 4];
  sdata[tid] += sdata[tid + 2];
  sdata[tid] += sdata[tid + 1];
}
// step 5
__global__ void reduce_unroll_last_loop_iteration(DataType *in, DataType *out,
                                                  int len)
{
  __shared__ DataType sdata[BLOCK_SIZE];
  int tid = threadIdx.x;
  int i = blockIdx.x * blockDim.x * 2 + threadIdx.x;
  sdata[tid] = in[i] + in[i + blockDim.x];
  __syncthreads();

  for (int s = blockDim.x / 2; s > 64; s >>= 1)
  {
    if (tid < s)
    {
      sdata[tid] += sdata[tid + s];
      __syncthreads();
    }
  }

  if (tid < 64)
  {
    warpReduce1(sdata, tid);
  }

  if (tid == 0)
  {
    atomicAdd(out, sdata[0]);
  }
}

__device__ void warpReduce2(volatile DataType *sdata, int tid)
{

  if constexpr (BLOCK_SIZE >= 128)
    sdata[tid] += sdata[tid + 64];
  if constexpr (BLOCK_SIZE >= 64)
    sdata[tid] += sdata[tid + 32];
  if constexpr (BLOCK_SIZE >= 32)
    sdata[tid] += sdata[tid + 16];
  if constexpr (BLOCK_SIZE >= 16)
    sdata[tid] += sdata[tid + 8];
  if constexpr (BLOCK_SIZE >= 8)
    sdata[tid] += sdata[tid + 4];
  if constexpr (BLOCK_SIZE >= 4)
    sdata[tid] += sdata[tid + 2];
  if constexpr (BLOCK_SIZE >= 2)
    sdata[tid] += sdata[tid + 1];
}
// step 6
__global__ void reduce_complete_unroll(DataType *in, DataType *out,
                                       int len)
{
  // TODO
  __shared__ DataType sdata[BLOCK_SIZE];
  int tid = threadIdx.x;
  int i = blockIdx.x * blockDim.x * 2 + threadIdx.x;
  sdata[tid] = in[i] + in[i + blockDim.x];
  __syncthreads();

  if constexpr (BLOCK_SIZE >= 512)
  {
    if (tid < 256)
      sdata[tid] += sdata[tid + 256];
    __syncthreads();
  }
  if constexpr (BLOCK_SIZE >= 256)
  {
    if (tid < 128)
      sdata[tid] += sdata[tid + 128];
    __syncthreads();
  }

  if (tid < 64)
  {
    warpReduce2(sdata, tid);
  }

  if (tid == 0)
  {
    atomicAdd(out, sdata[0]);
  }
}

// step 7
__global__ void reduce_algorithm_cascading(DataType *in, DataType *out,
                                           int len)
{
  // TODO
  __shared__ DataType sdata[BLOCK_SIZE];
  int tid = threadIdx.x;
  int i = blockIdx.x * blockDim.x * 2 + threadIdx.x;
  int grid_size = BLOCK_SIZE * 2 * gridDim.x;
  sdata[tid] = 0;
  while (i < (1 << 27))
  {
    sdata[tid] += in[i] + in[i + BLOCK_SIZE];
    i += grid_size;
  }
  __syncthreads();

  if constexpr (BLOCK_SIZE >= 512)
  {
    if (tid < 256)
      sdata[tid] += sdata[tid + 256];
    __syncthreads();
  }
  if constexpr (BLOCK_SIZE >= 256)
  {
    if (tid < 128)
      sdata[tid] += sdata[tid + 128];
    __syncthreads();
  }

  if (tid < 64)
  {
    warpReduce2(sdata, tid);
  }

  if (tid == 0)
  {
    atomicAdd(out, sdata[0]);
  }
}

// step 8
__global__ void reduce_use_warp_shuffles(DataType *in, DataType *out,
                                         int len)
{
  // TODO
  __shared__ DataType sdata[BLOCK_SIZE];
  int tid = threadIdx.x;
  int i = blockIdx.x * blockDim.x * 2 + threadIdx.x;
  sdata[tid] = in[i] + in[i + blockDim.x];
  __syncthreads();

  for (int s = blockDim.x / 2; s > 64; s >>= 1)
  {
    if (tid < s)
    {
      sdata[tid] += sdata[tid + s];
    }
    __syncthreads();
  }
  DataType sum;
  if (tid < 64)
  {
    sum = sdata[tid] + sdata[tid + 64];
    for (int s = 32; s > 0; s >>= 1)
    {
      sum += __shfl_down(sum, s, 0xffffffff);
    }
  }

  if (tid == 0)
  {
    atomicAdd(out, sum);
  }
}

int main(int argc, char *argv[])
{
  enum Mode
  {
    BASELINE = 0,
    AVOID_DIVERGENT_BRANCHING = 1,
    AVOID_BANK_CONFLICTS = 2,
    FIRST_ADD_DURING_LOAD = 3,
    UNROLL_LAST_LOOP_ITERATION = 4,
    COMPLETE_UNROLL = 5,
    ALGORITHM_CASCADING = 6,
    USE_WARP_SHUFFLES = 7
  } mode;
  if (argc < 2)
  {
    mode = (Mode)0;
  }
  else
  {
    sscanf(argv[1], "%d", &mode);
  }
  printf("mode = %d\n", mode);

  constexpr auto N = (1 << 27);
  constexpr auto SIZE = N * sizeof(DataType);

  vector<DataType> vec(N);
  for (int i = 0; i < N; ++i)
  {
    vec[i] = i;
  }

  DataType *in;
  DataType *out;
  DataType sum = 0;

  hipError_t hip_ret;

  hip_ret = hipMalloc(&in, SIZE);
  hip_ret = hipMalloc(&out, sizeof(DataType));

  hip_ret = hipMemcpy(in, vec.data(), SIZE, hipMemcpyHostToDevice);
  hip_ret = hipMemcpy(out, &sum, sizeof(DataType), hipMemcpyHostToDevice);

  constexpr auto N_BLOCKS = (N + BLOCK_SIZE - 1) / BLOCK_SIZE;

  switch (mode)
  {

  case AVOID_DIVERGENT_BRANCHING:
    hipLaunchKernelGGL(reduce_avoid_divergent_branching, dim3(N_BLOCKS),
                       dim3(BLOCK_SIZE), 0, 0, in, out, N);
    break;

  case AVOID_BANK_CONFLICTS:
    hipLaunchKernelGGL(reduce_avoid_bank_conflicts, dim3(N_BLOCKS),
                       dim3(BLOCK_SIZE), 0, 0, in, out, N);
    break;

  case FIRST_ADD_DURING_LOAD:
    hipLaunchKernelGGL(reduce_first_add_during_load, dim3(N_BLOCKS / 2),
                       dim3(BLOCK_SIZE), 0, 0, in, out, N);
    break;

  case UNROLL_LAST_LOOP_ITERATION:
    hipLaunchKernelGGL(reduce_unroll_last_loop_iteration, dim3(N_BLOCKS / 2),
                       dim3(BLOCK_SIZE), 0, 0, in, out, N);
    break;

  case COMPLETE_UNROLL:
    hipLaunchKernelGGL(reduce_complete_unroll, dim3(N_BLOCKS / 2),
                       dim3(BLOCK_SIZE), 0, 0, in, out, N);
    break;

  case ALGORITHM_CASCADING:
    hipLaunchKernelGGL(reduce_algorithm_cascading, dim3(N_BLOCKS / 2),
                       dim3(BLOCK_SIZE), 0, 0, in, out, N);
    break;

  case USE_WARP_SHUFFLES:
    hipLaunchKernelGGL(reduce_use_warp_shuffles, dim3(N_BLOCKS / 2),
                       dim3(BLOCK_SIZE), 0, 0, in, out, N);
    break;

  case BASELINE:
  default:
    hipLaunchKernelGGL(reduce_baseline, dim3(N_BLOCKS), dim3(BLOCK_SIZE), 0, 0,
                       in, out, N);
    break;
  }
  hip_ret = hipDeviceSynchronize();

  hip_ret = hipMemcpy(&sum, out, sizeof(DataType), hipMemcpyDeviceToHost);

  printf("sum of %d numbers: %lld\n", N, sum);

  hip_ret = hipFree(in);
  hip_ret = hipFree(out);
  check_hip_error(hip_ret);

  return 0;
}
