// This file is part of PlaneSweepLib (PSL)

// Copyright 2016 Christian Haene (ETH Zuerich)

// PSL is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// PSL is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with PSL.  If not, see <http://www.gnu.org/licenses/>.

#include <vector>

#include <psl/cudaCommon.h>
#include <psl/deviceBuffer.h>
#include <psl/deviceBuffer.cuh>
#include <psl/deviceImage.h>
#include <psl/deviceImage.cuh>


namespace PSL_CUDA
{
    namespace CudaFishEyePlaneSweepDeviceCode
    {
        __forceinline__ __device__ float computeWarpedGrayscaleTexturePixel(int x , int y, int width, int height,
                                                                            float krefinv11, float krefinv13, float krefinv22, float krefinv23, float xiref,
                                                                            float kother11, float kother13, float kother22, float kother23, float xiother,
                                                                            float rt11, float rt12, float rt13, float rt21, float rt22, float rt23, float rt31, float rt32, float rt33);
        __forceinline__ __device__ float computeZNCC(float normalizer, float ref, float refSqr, float other, float otherSqr, float prod);


        texture<uchar4, 2, cudaReadModeNormalizedFloat> planeSweepColorTexture;
        texture<uchar1, 2, cudaReadModeNormalizedFloat> planeSweepGrayscaleTexture;
        texture<unsigned char, 2> planeSweepGrayscaleTextureNonInterp;
        texture<float, 2> planeSweepLUTTexture;
        texture<float, 2> planeSweepCostTexture;

        cudaChannelFormatDesc planeSweepColorChannelDesc;
        cudaChannelFormatDesc planeSweepGrayscaleChannelDesc;
        cudaChannelFormatDesc planeSweepCostChannelDesc;
        cudaChannelFormatDesc planeSweepLUTChannelDesc;

        bool planeSweepTexturesInitialized = false;

        const int PLANE_SWEEP_TILE_WIDTH = 32;
        const int PLANE_SWEEP_TILE_HEIGHT = 8;

        const int PLANE_SWEEP_BOX_FILTER_NUM_THREADS = 128;
        const int PLANE_SWEEP_BOX_FILTER_ROWS_PER_THREAD = 25;

        void planeSweepInitTexturing()
        {
            if (!planeSweepTexturesInitialized)
            {
                // textures for image warping
                planeSweepColorChannelDesc = cudaCreateChannelDesc(8,8,8,8,cudaChannelFormatKindUnsigned);
                planeSweepGrayscaleChannelDesc = cudaCreateChannelDesc(8,0,0,0,cudaChannelFormatKindUnsigned);

                planeSweepColorTexture.addressMode[0] = cudaAddressModeWrap;
                planeSweepColorTexture.addressMode[1] = cudaAddressModeWrap;
                planeSweepColorTexture.filterMode = cudaFilterModeLinear;
                planeSweepColorTexture.normalized = true;

                planeSweepGrayscaleTexture.addressMode[0] = cudaAddressModeWrap;
                planeSweepGrayscaleTexture.addressMode[1] = cudaAddressModeWrap;
                planeSweepGrayscaleTexture.filterMode = cudaFilterModeLinear;
                planeSweepGrayscaleTexture.normalized = true;

                planeSweepGrayscaleTextureNonInterp.addressMode[0] = cudaAddressModeWrap;
                planeSweepGrayscaleTextureNonInterp.addressMode[1] = cudaAddressModeWrap;
                planeSweepGrayscaleTextureNonInterp.filterMode = cudaFilterModePoint;
                planeSweepGrayscaleTextureNonInterp.normalized = false;

                planeSweepLUTTexture.addressMode[0] = cudaAddressModeWrap;
                planeSweepLUTTexture.addressMode[1] = cudaAddressModeWrap;
                planeSweepLUTTexture.filterMode = cudaFilterModePoint;
                planeSweepLUTTexture.normalized = false;

                planeSweepLUTChannelDesc = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);

                // textures for box filtering costs
                planeSweepCostChannelDesc = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);

                planeSweepCostTexture.addressMode[0] = cudaAddressModeClamp;
                planeSweepCostTexture.addressMode[1] = cudaAddressModeClamp;
                planeSweepCostTexture.filterMode = cudaFilterModePoint;
                planeSweepCostTexture.normalized = false;

                planeSweepTexturesInitialized = true;
            }
        }


        __global__ void boxFilterCostsKernel(DeviceBuffer<float> filteredBuf, int radius_x, int radius_y)
        {
            // implemented according to "Stereo Imaging with CUDA" and lots of changes since initial implementation
            extern __shared__ float colSum[];

            const int X =  blockIdx.x*PLANE_SWEEP_BOX_FILTER_NUM_THREADS + threadIdx.x - blockIdx.x*2*radius_x;
            const int Y =  blockIdx.y*PLANE_SWEEP_BOX_FILTER_ROWS_PER_THREAD;

            const int width = filteredBuf.getWidth();
            const int height = filteredBuf.getHeight();

            if (X < (width + 2*radius_x) && Y < height)
            {
                int x_cost = X - radius_x;
                colSum[threadIdx.x] = 0;

                int y_cost = Y - radius_y;
                for (int i = 0; i <= 2*radius_y; i++)
                {
                    colSum[threadIdx.x] += tex2D(planeSweepCostTexture, x_cost, y_cost);
                    y_cost++;
                }
                __syncthreads();


                if (threadIdx.x + 2*radius_x < PLANE_SWEEP_BOX_FILTER_NUM_THREADS && X < width && Y < height)
                {
                    float sum = 0;
                    for (int i = 0; i <= 2*radius_x; i++)
                    {
                        sum = sum + colSum[i+threadIdx.x];
                    }
                    filteredBuf(X, Y) = sum;
                }
                __syncthreads();

                y_cost = Y - radius_y;
                for (int row = 1; row < PLANE_SWEEP_BOX_FILTER_ROWS_PER_THREAD && (Y + row < height); row++)
                {
                    colSum[threadIdx.x] -= tex2D(planeSweepCostTexture, x_cost, y_cost);
                    colSum[threadIdx.x] += tex2D(planeSweepCostTexture, x_cost, y_cost + 2*radius_y + 1);

                    y_cost++;
                    __syncthreads();

                    if (threadIdx.x + 2*radius_x < PLANE_SWEEP_BOX_FILTER_NUM_THREADS && X < width)
                    {
                        float sum = 0;
                        for (int i = 0; i <= 2*radius_x; i++)
                        {
                            sum += colSum[i+threadIdx.x];
                        }
                        filteredBuf(X,Y+row) = sum;
                    }
                    __syncthreads();
                }
            }
        }

        void planeSweepBoxFilterCosts(DeviceBuffer<float>& costBuf, DeviceBuffer<float>& filteredCostBuf, int radius_x, int radius_y)
        {
            // Bind texture
            PSL_CUDA_CHECKED_CALL( cudaBindTexture2D(0, planeSweepCostTexture, costBuf.getAddr(), planeSweepCostChannelDesc, costBuf.getWidth(), costBuf.getHeight(), costBuf.getPitch()); )

            const int sharedMemSize = PLANE_SWEEP_BOX_FILTER_NUM_THREADS*sizeof(float);

            dim3 gridDim(getNumTiles(filteredCostBuf.getWidth(), PLANE_SWEEP_BOX_FILTER_NUM_THREADS - 2*radius_x), getNumTiles(filteredCostBuf.getHeight(), PLANE_SWEEP_BOX_FILTER_ROWS_PER_THREAD));
            dim3 blockDim(PLANE_SWEEP_BOX_FILTER_NUM_THREADS, 1);

            // run box filtering kernel
            boxFilterCostsKernel<<<gridDim, blockDim, sharedMemSize>>>(filteredCostBuf, radius_x, radius_y);
            PSL_CUDA_CHECK_ERROR

            // unbind texture
            PSL_CUDA_CHECKED_CALL( cudaUnbindTexture(planeSweepCostTexture); )
        }


        __global__ void planeSweepBoxFilterImageAndSqrImageKernel(DeviceBuffer<float> boxFilterBuf, DeviceBuffer<float> boxFilterSqrBuf, int radius_x, int radius_y)
        {
            // implemented according to "Stereo Imaging with CUDA" and lots of changes since the initial implementation

            extern __shared__ float colSum[];

            const int sqrOffset = PLANE_SWEEP_BOX_FILTER_NUM_THREADS;

            const int X = blockIdx.x*PLANE_SWEEP_BOX_FILTER_NUM_THREADS + threadIdx.x - blockIdx.x*2*radius_x;
            const int Y = blockIdx.y*PLANE_SWEEP_BOX_FILTER_ROWS_PER_THREAD;

            const int width = boxFilterBuf.getWidth();
            const int height = boxFilterBuf.getHeight();

            if (X < (width + 2*radius_x) && Y < height)
            {
                int x_img = X - radius_x;
                colSum[threadIdx.x] = 0;
                colSum[sqrOffset + threadIdx.x] = 0;

                int y_img = Y - radius_y;
                for (int i = 0; i <= 2*radius_y; i++)
                {
                    const float val = tex2D(planeSweepGrayscaleTextureNonInterp, x_img, y_img);
                    colSum[threadIdx.x] += val;
                    colSum[sqrOffset + threadIdx.x] += val*val;

                    y_img++;
                }
                __syncthreads();

                if (threadIdx.x + 2*radius_x < PLANE_SWEEP_BOX_FILTER_NUM_THREADS && X < width && Y < height)
                {
                    float sum = 0;
                    float sqrSum = 0;
                    for (int i = 0; i <= 2*radius_x; i++)
                    {
                        sum += colSum[i+threadIdx.x];
                        sqrSum += colSum[sqrOffset + i + threadIdx.x];
                    }
                    boxFilterBuf(X,Y) = sum;
                    boxFilterSqrBuf(X,Y) = sqrSum;
                }

                __syncthreads();

                y_img = Y - radius_y;
                for (int row = 1; row < PLANE_SWEEP_BOX_FILTER_ROWS_PER_THREAD && (row + Y) < height; row++)
                {
                    const float subVal = tex2D(planeSweepGrayscaleTextureNonInterp, x_img, y_img);
                    colSum[threadIdx.x] -= subVal;
                    colSum[sqrOffset + threadIdx.x] -= subVal*subVal;
                    const float addVal = tex2D(planeSweepGrayscaleTextureNonInterp, x_img, y_img + 2*radius_y + 1);
                    colSum[threadIdx.x] += addVal;
                    colSum[sqrOffset + threadIdx.x] += addVal*addVal;
                    y_img++;
                    __syncthreads();

                    if (threadIdx.x + 2*radius_x < PLANE_SWEEP_BOX_FILTER_NUM_THREADS && X < width)
                    {
                        float sum = 0;
                        float sqrSum = 0;
                        for (int i = 0; i <= 2*radius_x; i++)
                        {
                            sum += colSum[i+threadIdx.x];
                            sqrSum += colSum[sqrOffset + i + threadIdx.x];
                        }
                        boxFilterBuf(X,Y+row) = sum;
                        boxFilterSqrBuf(X,Y+row) = sqrSum;
                    }
                    __syncthreads();
                }
            }
        }

        void planeSweepBoxFilterImageAndSqrImage(const DeviceImage& refImg,
                                                 DeviceBuffer<float>& boxFilterBuf, DeviceBuffer<float>& boxFilterSqrBuf,
                                                 DeviceBuffer<float>& tempBuf, DeviceBuffer<float>& tempSqrBuf, int radius_x, int radius_y)
        {

            const int width = refImg.getWidth();
            const int height = refImg.getHeight();

            // bind texture
            PSL_CUDA_CHECKED_CALL( cudaBindTexture2D(0, planeSweepGrayscaleTextureNonInterp, refImg.getAddr(), planeSweepGrayscaleChannelDesc, refImg.getWidth(), refImg.getHeight(), refImg.getPitch()); )

            const int sharedMemSize = 2*(PLANE_SWEEP_BOX_FILTER_NUM_THREADS)*sizeof(float);

            dim3 gridDim(getNumTiles(width, PLANE_SWEEP_BOX_FILTER_NUM_THREADS - 2*radius_x), getNumTiles(height, PLANE_SWEEP_BOX_FILTER_ROWS_PER_THREAD));
            dim3 blockDim(PLANE_SWEEP_BOX_FILTER_NUM_THREADS, 1);
            planeSweepBoxFilterImageAndSqrImageKernel<<<gridDim, blockDim, sharedMemSize>>>(boxFilterBuf, boxFilterSqrBuf, radius_x, radius_y);
            PSL_CUDA_CHECK_ERROR

            PSL_CUDA_CHECKED_CALL(cudaUnbindTexture(planeSweepGrayscaleTextureNonInterp))
        }



        __global__ void planeSweepZNCCKernel(float krefinv11, float krefinv13, float krefinv22, float krefinv23, float xiref,
                                             float kother11, float kother13, float kother22, float kother23, float xiother,
                                             float rt11, float rt12, float rt13, float rt21, float rt22, float rt23, float rt31, float rt32, float rt33,
                                             DeviceBuffer<float> refFiltered, DeviceBuffer<float> refSqrFiltered,
                                             DeviceBuffer<float> costBuffer, int radius_x, int radius_y, float normalizerZNCC)
        {
            // implemented according to "Stereo Imaging with CUDA" and lots of changes since initial implementation

            const int sqrOffset = PLANE_SWEEP_BOX_FILTER_NUM_THREADS;
            const int prodOffset = 2*sqrOffset;

            extern __shared__ float colSum[];

            const int X = (blockIdx.x*PLANE_SWEEP_BOX_FILTER_NUM_THREADS + threadIdx.x)-blockIdx.x*2*radius_x;
            const int Y = blockIdx.y*PLANE_SWEEP_BOX_FILTER_ROWS_PER_THREAD;

            const int width = costBuffer.getWidth();
            const int height = costBuffer.getHeight();

            if (X < (width + 2*radius_x) && Y < height)
            {
                int x_img = X - radius_x;
                colSum[threadIdx.x] = 0;
                colSum[sqrOffset + threadIdx.x] = 0;
                colSum[prodOffset + threadIdx.x] = 0;

                int y_img = Y - radius_y;
                for (int i = 0; i <= 2*radius_y; i++)
                {
                    const float val = tex2D(planeSweepGrayscaleTextureNonInterp, x_img, y_img);
                    const float otherVal = computeWarpedGrayscaleTexturePixel(x_img, y_img, width, height,
                                                                              krefinv11, krefinv13, krefinv22, krefinv23, xiref,
                                                                              kother11, kother13, kother22, kother23, xiother,
                                                                              rt11, rt12, rt13, rt21, rt22, rt23, rt31, rt32, rt33);
                    colSum[threadIdx.x] += otherVal;
                    colSum[sqrOffset + threadIdx.x] += otherVal*otherVal;
                    colSum[prodOffset + threadIdx.x] += val*otherVal;

                    y_img++;
                }
                __syncthreads();

                if (threadIdx.x + 2*radius_x < PLANE_SWEEP_BOX_FILTER_NUM_THREADS && X < width && Y < height)
                {
                    float other = 0;
                    float otherSqr = 0;
                    float prod = 0;
                    for (int i = 0; i <= 2*radius_x; i++)
                    {
                        other += colSum[i+threadIdx.x];
                        otherSqr += colSum[sqrOffset + i + threadIdx.x];
                        prod += colSum[prodOffset + i + threadIdx.x];
                    }
                    const float ref = refFiltered(X,Y);
                    const float refSqr = refSqrFiltered(X,Y);

                    costBuffer(X,Y) = computeZNCC(normalizerZNCC, ref, refSqr, other, otherSqr, prod);
                }

                __syncthreads();

                y_img = Y - radius_y;
                for (int row = 1; row < PLANE_SWEEP_BOX_FILTER_ROWS_PER_THREAD && (row + Y < height); row++)
                {
                    const float subVal = tex2D(planeSweepGrayscaleTextureNonInterp, x_img, y_img);
                    const float subOtherVal = computeWarpedGrayscaleTexturePixel(x_img, y_img, width, height,
                                                                                 krefinv11, krefinv13, krefinv22, krefinv23, xiref,
                                                                                 kother11, kother13, kother22, kother23, xiother,
                                                                                 rt11, rt12, rt13, rt21, rt22, rt23, rt31, rt32, rt33);
                    colSum[threadIdx.x] -= subOtherVal;
                    colSum[sqrOffset + threadIdx.x] -= subOtherVal*subOtherVal;
                    colSum[prodOffset + threadIdx.x] -= subOtherVal*subVal;
                    const float addVal = tex2D(planeSweepGrayscaleTextureNonInterp, x_img, y_img + 2*radius_y + 1);
                    const float addOtherVal = computeWarpedGrayscaleTexturePixel( x_img, y_img + 2*radius_y + 1, width, height,
                                                                                  krefinv11, krefinv13, krefinv22, krefinv23, xiref,
                                                                                  kother11, kother13, kother22, kother23, xiother,
                                                                                  rt11, rt12, rt13, rt21, rt22, rt23, rt31, rt32, rt33);
                    colSum[threadIdx.x] += addOtherVal;
                    colSum[sqrOffset + threadIdx.x] += addOtherVal*addOtherVal;
                    colSum[prodOffset + threadIdx.x] += addOtherVal*addVal;
                    y_img++;

                    __syncthreads();

                    if (threadIdx.x + 2*radius_x < PLANE_SWEEP_BOX_FILTER_NUM_THREADS && X < width)
                    {
                        float other = 0;
                        float otherSqr = 0;
                        float prod = 0;
                        for (int i = 0; i <= 2*radius_x; i++)
                        {
                            other += colSum[i+threadIdx.x];
                            otherSqr += colSum[sqrOffset + i + threadIdx.x];
                            prod += colSum[prodOffset + i + threadIdx.x];
                        }
                        const float ref = refFiltered(X,Y+row);
                        const float refSqr = refSqrFiltered(X,Y+row);

                        costBuffer(X, Y+row) = computeZNCC(normalizerZNCC, ref, refSqr, other, otherSqr, prod);
                    }
                    __syncthreads();
                }
            }
        }

        void planeSweepWarpZNCC(const DeviceImage &otherImg, float* RT,
                                const float* KRefInv, const float XiRef,
                                const float* Kother, const float XiOther,
                                const DeviceImage &refImg,
                                DeviceBuffer<float>& refImgBoxFilterBuf, DeviceBuffer<float>& refImgSqrBoxFilterBuf,
                                DeviceBuffer<float>& costBuf, int radius_x, int radius_y)
        {
            // bind textures
            PSL_CUDA_CHECKED_CALL( cudaBindTexture2D(0, planeSweepGrayscaleTexture, otherImg.getAddr(), planeSweepGrayscaleChannelDesc, otherImg.getWidth(), otherImg.getHeight(), otherImg.getPitch()); )
            PSL_CUDA_CHECKED_CALL( cudaBindTexture2D(0, planeSweepGrayscaleTextureNonInterp, refImg.getAddr(), planeSweepGrayscaleChannelDesc, refImg.getWidth(), refImg.getHeight(), refImg.getPitch()); )

            // compute kernel config
            const int sharedMemSize = 3*(PLANE_SWEEP_BOX_FILTER_NUM_THREADS)*sizeof(float);
            dim3 gridDim(getNumTiles(refImg.getWidth(), PLANE_SWEEP_BOX_FILTER_NUM_THREADS-2*radius_x), getNumTiles(refImg.getHeight(), PLANE_SWEEP_BOX_FILTER_ROWS_PER_THREAD));
            dim3 blockDim(PLANE_SWEEP_BOX_FILTER_NUM_THREADS, 1);

            const float normalizerZNCC = 1.0f/((float)(2*radius_x+1)*(2*radius_y+1));

            // run filter kernel
            planeSweepZNCCKernel<<<gridDim, blockDim, sharedMemSize>>>(KRefInv[0], KRefInv[2], KRefInv[4], KRefInv[5], XiRef,
                                                                       Kother[0], Kother[2], Kother[4], Kother[5], XiOther,
                                                                       RT[0], RT[1], RT[2], RT[3], RT[4], RT[5], RT[6], RT[7], RT[8],
                                                                       refImgBoxFilterBuf, refImgSqrBoxFilterBuf,
                                                                       costBuf, radius_x, radius_y, normalizerZNCC);
            PSL_CUDA_CHECK_ERROR

            // unbind textures
            PSL_CUDA_CHECKED_CALL( cudaUnbindTexture(planeSweepGrayscaleTexture) )
            PSL_CUDA_CHECKED_CALL( cudaUnbindTexture(planeSweepGrayscaleTextureNonInterp) )
        }



        __global__ void planeSweepZNCCAccumKernel(float krefinv11, float krefinv13, float krefinv22, float krefinv23, float xiref,
                                                  float kother11, float kother13, float kother22, float kother23, float xiother,
                                                  float rt11, float rt12, float rt13, float rt21, float rt22, float rt23, float rt31, float rt32, float rt33,
                                                  DeviceBuffer<float> refFiltered, DeviceBuffer<float> refSqrFiltered,
                                                  DeviceBuffer<float> costAccumBuffer, float accumScale, int radius_x, int radius_y, float normalizerZNCC)
        {
            // implemented according to "Stereo Imaging with CUDA" and lots of changes since initial implementation

            const int sqrOffset = PLANE_SWEEP_BOX_FILTER_NUM_THREADS;
            const int prodOffset = 2*sqrOffset;

            extern __shared__ float colSum[];

            const int X = (blockIdx.x*PLANE_SWEEP_BOX_FILTER_NUM_THREADS + threadIdx.x)-blockIdx.x*2*radius_x;
            const int Y = blockIdx.y*PLANE_SWEEP_BOX_FILTER_ROWS_PER_THREAD;

            const int width = costAccumBuffer.getWidth();
            const int height = costAccumBuffer.getHeight();

            if (X < (width + 2*radius_x) && Y < height)
            {
                int x_img = X - radius_x;
                colSum[threadIdx.x] = 0;
                colSum[sqrOffset + threadIdx.x] = 0;
                colSum[prodOffset + threadIdx.x] = 0;

                int y_img = Y - radius_y;
                for (int i = 0; i <= 2*radius_y; i++)
                {
                    const float val = tex2D(planeSweepGrayscaleTextureNonInterp, x_img, y_img);
                    const float otherVal = computeWarpedGrayscaleTexturePixel(x_img, y_img, width, height,
                                                                              krefinv11, krefinv13, krefinv22, krefinv23, xiref,
                                                                              kother11, kother13, kother22, kother23, xiother,
                                                                              rt11, rt12, rt13, rt21, rt22, rt23, rt31, rt32, rt33);
                    colSum[threadIdx.x] += otherVal;
                    colSum[sqrOffset + threadIdx.x] += otherVal*otherVal;
                    colSum[prodOffset + threadIdx.x] += val*otherVal;

                    y_img++;
                }
                __syncthreads();

                if (threadIdx.x + 2*radius_x < PLANE_SWEEP_BOX_FILTER_NUM_THREADS && X < width && Y < height)
                {
                    float other = 0;
                    float otherSqr = 0;
                    float prod = 0;
                    for (int i = 0; i <= 2*radius_x; i++)
                    {
                        other += colSum[i+threadIdx.x];
                        otherSqr += colSum[sqrOffset + i + threadIdx.x];
                        prod += colSum[prodOffset + i + threadIdx.x];
                    }
                    const float ref = refFiltered(X,Y);
                    const float refSqr = refSqrFiltered(X,Y);

                    costAccumBuffer(X,Y) += accumScale*computeZNCC(normalizerZNCC, ref, refSqr, other, otherSqr, prod);
                }
                __syncthreads();

                y_img = Y - radius_y;
                for (int row = 1; row < PLANE_SWEEP_BOX_FILTER_ROWS_PER_THREAD && (row + Y < height); row++)
                {
                    const float subVal = tex2D(planeSweepGrayscaleTextureNonInterp, x_img, y_img);
                    const float subOtherVal = computeWarpedGrayscaleTexturePixel(x_img, y_img, width, height,
                                                                                 krefinv11, krefinv13, krefinv22, krefinv23, xiref,
                                                                                 kother11, kother13, kother22, kother23, xiother,
                                                                                 rt11, rt12, rt13, rt21, rt22, rt23, rt31, rt32, rt33);
                    colSum[threadIdx.x] -= subOtherVal;
                    colSum[sqrOffset + threadIdx.x] -= subOtherVal*subOtherVal;
                    colSum[prodOffset + threadIdx.x] -= subOtherVal*subVal;
                    const float addVal = tex2D(planeSweepGrayscaleTextureNonInterp, x_img, y_img + 2*radius_y + 1);
                    const float addOtherVal = computeWarpedGrayscaleTexturePixel( x_img, y_img + 2*radius_y + 1, width, height,
                                                                                  krefinv11, krefinv13, krefinv22, krefinv23, xiref,
                                                                                  kother11, kother13, kother22, kother23, xiother,
                                                                                  rt11, rt12, rt13, rt21, rt22, rt23, rt31, rt32, rt33);
                    colSum[threadIdx.x] += addOtherVal;
                    colSum[sqrOffset + threadIdx.x] += addOtherVal*addOtherVal;
                    colSum[prodOffset + threadIdx.x] += addOtherVal*addVal;
                    y_img++;

                    __syncthreads();

                    if (threadIdx.x + 2*radius_x < PLANE_SWEEP_BOX_FILTER_NUM_THREADS && X < width)
                    {
                        float other = 0;
                        float otherSqr = 0;
                        float prod = 0;
                        for (int i = 0; i <= 2*radius_x; i++)
                        {
                            other += colSum[i+threadIdx.x];
                            otherSqr += colSum[sqrOffset + i + threadIdx.x];
                            prod += colSum[prodOffset + i + threadIdx.x];
                        }
                        const float ref = refFiltered(X,Y+row);
                        const float refSqr = refSqrFiltered(X,Y+row);

                        costAccumBuffer(X, Y+row) += accumScale*computeZNCC(normalizerZNCC, ref, refSqr, other, otherSqr, prod);
                    }
                    __syncthreads();
                }
            }
        }

        void planeSweepWarpZNCCAccum(const DeviceImage& otherImg, float* RT,
                                     const float* KRefInv, const float XiRef,
                                     const float* Kother, const float XiOther,
                                     const DeviceImage& refImg,
                                     DeviceBuffer<float>& refImgBoxFilterBuf, DeviceBuffer<float>& refImgSqrBoxFilterBuf,
                                     float accumScale, DeviceBuffer<float>& costAccumBuf, int radius_x, int radius_y)
        {
            // bind textures
            PSL_CUDA_CHECKED_CALL( cudaBindTexture2D(0, planeSweepGrayscaleTexture, otherImg.getAddr(), planeSweepGrayscaleChannelDesc, otherImg.getWidth(), otherImg.getHeight(), otherImg.getPitch()); )
            PSL_CUDA_CHECKED_CALL( cudaBindTexture2D(0, planeSweepGrayscaleTextureNonInterp, refImg.getAddr(), planeSweepGrayscaleChannelDesc, refImg.getWidth(), refImg.getHeight(), refImg.getPitch()); )

            // compute kernel config
            const int sharedMemSize = 3*(PLANE_SWEEP_BOX_FILTER_NUM_THREADS)*sizeof(float);
            dim3 gridDim(getNumTiles(refImg.getWidth(), PLANE_SWEEP_BOX_FILTER_NUM_THREADS-2*radius_x), getNumTiles(refImg.getHeight(), PLANE_SWEEP_BOX_FILTER_ROWS_PER_THREAD));
            dim3 blockDim(PLANE_SWEEP_BOX_FILTER_NUM_THREADS, 1);

            const float normalizerZNCC = 1.0f/((float)(2*radius_x+1)*(2*radius_y+1));

            // run filter kernel
            planeSweepZNCCAccumKernel<<<gridDim, blockDim, sharedMemSize>>>(KRefInv[0], KRefInv[2], KRefInv[4], KRefInv[5], XiRef,
                                                                            Kother[0], Kother[2], Kother[4], Kother[5], XiOther,
                                                                            RT[0], RT[1], RT[2], RT[3], RT[4], RT[5], RT[6], RT[7], RT[8],
                                                                            refImgBoxFilterBuf, refImgSqrBoxFilterBuf,
                                                                            costAccumBuf, accumScale, radius_x, radius_y, normalizerZNCC);
            PSL_CUDA_CHECK_ERROR

            // unbind textures
            PSL_CUDA_CHECKED_CALL( cudaUnbindTexture(planeSweepGrayscaleTexture) )
            PSL_CUDA_CHECKED_CALL( cudaUnbindTexture(planeSweepGrayscaleTextureNonInterp) )
        }


        __global__ void planeSweepWarpADGrayscaleKernel(const int srcImgWidth, const int srcImgHeight,
                                                        float krefinv11, float krefinv13, float krefinv22, float krefinv23, float xiref,
                                                        float kother11, float kother13, float kother22, float kother23, float xiother,
                                                        float rt11, float rt12, float rt13, float rt21, float rt22, float rt23, float rt31, float rt32, float rt33,
                                                        DeviceImage refImg, DeviceBuffer<float> costBuf)
        {
            // get position of outupt
            const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            const int width = refImg.getWidth();
            const int height = refImg.getHeight();

            if (x < width && y < height)
            {
                // apply Krefinv
                float mx = krefinv11*x + krefinv13;
                float my = krefinv22*y + krefinv23;

                // apply h^-1 of camera model
                float mxPmySqr = mx*mx + my*my;
                float D = 1 + (1 - xiref*xiref)*mxPmySqr;

                if (D < 0)
                    D = -D;

                float fact = (xiref + sqrtf(D))/(mxPmySqr + 1);

                float xx = fact*mx;
                float yy = fact*my;
                float zz = fact - xiref;

                // apply rt
                float xxw = rt11*xx + rt12*yy + rt13*zz;
                float yyw = rt21*xx + rt22*yy + rt23*zz;
                float zzw = rt31*xx + rt32*yy + rt33*zz;

                // apply h
                float length = sqrtf(xxw*xxw + yyw*yyw + zzw*zzw);
                float xxxw = xxw/length;
                float yyyw = yyw/length;
                float zzzw = zzw/length + xiother;

                float xmw = xxxw/zzzw;
                float ymw = yyyw/zzzw;

                // apply Kother

                float xw = kother11*xmw + kother13;
                float yw = kother22*ymw + kother23;

                const float u = (xw+0.5f) / (float) srcImgWidth;
                const float v = (yw+0.5f) / (float) srcImgHeight;

                const float1 pix = tex2D(planeSweepGrayscaleTexture, u, v);

                const float i = __saturatef(fabs(pix.x))*255;

                // ad
                float id = fabs((float)(refImg(x,y) - i));
//                float id = fabs((float)(i));

                // accumulate
                costBuf(x,y) = id;
            }
        }

        void planeSweepWarpAD(const DeviceImage &srcImg, const float* RT,
                              const float* KRefInv, const float XiRef,
                              const float* Kother, const float XiOther,
                              const DeviceImage &refImg, DeviceBuffer<float>& costBuf)
        {
            dim3 gridDim(getNumTiles(refImg.getWidth(), PLANE_SWEEP_TILE_WIDTH), getNumTiles(refImg.getHeight(), PLANE_SWEEP_TILE_HEIGHT));
            dim3 blockDim(PLANE_SWEEP_TILE_WIDTH, PLANE_SWEEP_TILE_HEIGHT);

            // bind the grayscale texture
            PSL_CUDA_CHECKED_CALL( cudaBindTexture2D(0, planeSweepGrayscaleTexture, srcImg.getAddr(), planeSweepGrayscaleChannelDesc, srcImg.getWidth(), srcImg.getHeight(), srcImg.getPitch()); )

            planeSweepWarpADGrayscaleKernel<<<gridDim, blockDim>>>(srcImg.getWidth(), srcImg.getHeight(),
                                                                   KRefInv[0], KRefInv[2], KRefInv[4], KRefInv[5], XiRef,
                                                                   Kother[0], Kother[2], Kother[4], Kother[5], XiOther,
                                                                   RT[0], RT[1], RT[2], RT[3], RT[4], RT[5], RT[6], RT[7], RT[8],
                                                                   refImg, costBuf);
            PSL_CUDA_CHECK_ERROR
            PSL_CUDA_CHECKED_CALL( cudaUnbindTexture(planeSweepGrayscaleTexture); )
        }



        __global__ void planeSweepWarpADAccumGrayscaleKernel(const int srcImgWidth, const int srcImgHeight,
                                                             float krefinv11, float krefinv13, float krefinv22, float krefinv23, float xiref,
                                                             float kother11, float kother13, float kother22, float kother23, float xiother,
                                                             float rt11, float rt12, float rt13, float rt21, float rt22, float rt23, float rt31, float rt32, float rt33,
                                                             DeviceImage refImg, float accumScale, DeviceBuffer<float> costAccumBuf)
        {
            // get position of outupt
            const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            const int width = refImg.getWidth();
            const int height = refImg.getHeight();

            if (x < width && y < height)
            {
                // apply Krefinv
                float mx = krefinv11*x + krefinv13;
                float my = krefinv22*y + krefinv23;

                // apply h^-1 of camera model
                float mxPmySqr = mx*mx + my*my;
                float D = 1 + (1 - xiref*xiref)*mxPmySqr;

                if (D < 0)
                    D = -D;

                float fact = (xiref + sqrtf(D))/(mxPmySqr + 1);

                float xx = fact*mx;
                float yy = fact*my;
                float zz = fact - xiref;

                // apply rt
                float xxw = rt11*xx + rt12*yy + rt13*zz;
                float yyw = rt21*xx + rt22*yy + rt23*zz;
                float zzw = rt31*xx + rt32*yy + rt33*zz;

                // apply h
                float length = sqrtf(xxw*xxw + yyw*yyw + zzw*zzw);
                float xxxw = xxw/length;
                float yyyw = yyw/length;
                float zzzw = zzw/length + xiother;

                float xmw = xxxw/zzzw;
                float ymw = yyyw/zzzw;

                // apply Kother

                float xw = kother11*xmw + kother13;
                float yw = kother22*ymw + kother23;

                const float u = (xw+0.5f) / (float) srcImgWidth;
                const float v = (yw+0.5f) / (float) srcImgHeight;

                const float1 pix = tex2D(planeSweepGrayscaleTexture, u, v);

                const float i = __saturatef(fabs(pix.x))*255;

                // ad
                float id = fabs((float)(refImg(x,y) - i));
//                float id = fabs((float)(i));

                // accumulate
                costAccumBuf(x,y) += accumScale*id;
            }
        }

        void planeSweepWarpADAccum(DeviceImage& srcImg, const float* RT,
                                   const float* KRefInv, const float XiRef,
                                   const float* Kother, const float XiOther,
                                   DeviceImage& refImg,
                                   float accumScale, DeviceBuffer<float>& costAccumBuf)
        {
            dim3 gridDim(getNumTiles(refImg.getWidth(), PLANE_SWEEP_TILE_WIDTH), getNumTiles(refImg.getHeight(), PLANE_SWEEP_TILE_HEIGHT));
            dim3 blockDim(PLANE_SWEEP_TILE_WIDTH, PLANE_SWEEP_TILE_HEIGHT);

            // bind the grayscale texture
            PSL_CUDA_CHECKED_CALL( cudaBindTexture2D(0, planeSweepGrayscaleTexture, srcImg.getAddr(), planeSweepGrayscaleChannelDesc, srcImg.getWidth(), srcImg.getHeight(), srcImg.getPitch()); )

            planeSweepWarpADAccumGrayscaleKernel<<<gridDim, blockDim>>>(srcImg.getWidth(), srcImg.getHeight(),
                                                                        KRefInv[0], KRefInv[2], KRefInv[4], KRefInv[5], XiRef,
                                                                        Kother[0], Kother[2], Kother[4], Kother[5], XiOther,
                                                                        RT[0], RT[1], RT[2], RT[3], RT[4], RT[5], RT[6], RT[7], RT[8],
                                                                     refImg, accumScale, costAccumBuf);
            PSL_CUDA_CHECK_ERROR
            PSL_CUDA_CHECKED_CALL( cudaUnbindTexture(planeSweepGrayscaleTexture); )
        }


        __global__ void planeSweepUpdateBestKKernel(DeviceBuffer<float> newCosts, DeviceBuffer<float> bestCosts, DeviceBuffer<float> bestMin)
        {
            unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < newCosts.getWidth() && y < newCosts.getHeight())
            {
                const float newCost = newCosts(x,y);

                if (newCost > bestMin(x,y) && newCost < bestCosts(x,y))
                {
                    bestCosts(x,y) = newCost;
                }
            }
        }

        void updateBestK(DeviceBuffer<float>& newCostsBuf, DeviceBuffer<float>& bestCostsBuf, DeviceBuffer<float>& bestMinBuf)
        {
            dim3 gridDim(getNumTiles(bestCostsBuf.getWidth(), PLANE_SWEEP_TILE_WIDTH), getNumTiles(bestCostsBuf.getHeight(), PLANE_SWEEP_TILE_HEIGHT));
            dim3 blockDim(PLANE_SWEEP_TILE_WIDTH, PLANE_SWEEP_TILE_HEIGHT);

            planeSweepUpdateBestKKernel<<<gridDim, blockDim>>>(newCostsBuf, bestCostsBuf, bestMinBuf);
            PSL_CUDA_CHECK_ERROR
        }

        __global__ void planeSweepAccumCostKernelBestK(DeviceBuffer<float> accumBuf, DeviceBuffer<float> costBuf, DeviceBuffer<float> minCostBuf, float maxVal, float accumScale)
        {
            // get position of outupt
            const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < accumBuf.getWidth() && y < accumBuf.getHeight())
            {
                const float newVal = costBuf(x,y);
                if (newVal < maxVal)
                {
                    accumBuf(x,y) += accumScale*newVal;
                }
                else
                {
                    const float usedVal = minCostBuf(x,y);
                    accumBuf(x,y) += accumScale*usedVal;
                    costBuf(x,y) = usedVal;
                }
            }

        }

        void planeSweepAccumCostBestK(DeviceBuffer<float>& costAccumBuf, DeviceBuffer<float>& costBuf, DeviceBuffer<float>& minCostBuf, float maxVal, float accumScale)
        {
            dim3 gridDim(getNumTiles(costAccumBuf.getWidth(), PLANE_SWEEP_TILE_WIDTH), getNumTiles(costAccumBuf.getHeight(), PLANE_SWEEP_TILE_HEIGHT));
            dim3 blockDim(PLANE_SWEEP_TILE_WIDTH, PLANE_SWEEP_TILE_HEIGHT);

            planeSweepAccumCostKernelBestK<<<gridDim, blockDim>>>(costAccumBuf, costBuf, minCostBuf, maxVal, accumScale);
            PSL_CUDA_CHECK_ERROR
        }

        __global__ void planeSweepUpdateBestPlaneKernel(DeviceBuffer<float> newCosts, int currPlaneIndex, DeviceBuffer<float> bestPlaneCosts, DeviceBuffer<int> bestPlanes)
        {
            // get position of outupt
            unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < newCosts.getWidth() && y < newCosts.getHeight())
            {
                if (newCosts(x,y) <= bestPlaneCosts(x,y))
                {
                    bestPlaneCosts(x,y) = newCosts(x,y);
                    bestPlanes(x,y) = currPlaneIndex;
                }
            }

        }

        void planeSweepUpdateBestPlane(const DeviceBuffer<float>& newCosts, int width, int height, int currPlaneIndex,
                                       DeviceBuffer<float>& bestPlaneCosts, DeviceBuffer<int>& bestPlanes)
        {
            dim3 gridDim(getNumTiles(width, PLANE_SWEEP_TILE_WIDTH), getNumTiles(height, PLANE_SWEEP_TILE_HEIGHT));
            dim3 blockDim(PLANE_SWEEP_TILE_WIDTH, PLANE_SWEEP_TILE_HEIGHT);

            planeSweepUpdateBestPlaneKernel<<<gridDim, blockDim>>>(newCosts, currPlaneIndex, bestPlaneCosts, bestPlanes);
            PSL_CUDA_CHECK_ERROR
        }

        __global__ void planeSweepUpdateBestPlaneSubPixelKernel(DeviceBuffer<float> currCosts, DeviceBuffer<float> prev1, DeviceBuffer<float> prev2, int prevPlaneIdx,
                                                                DeviceBuffer<float> bestPlaneCosts, DeviceBuffer<int> bestPlanes, DeviceBuffer<float> subPixelPlaneOffsets)
        {
            // get position of outupt
            const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < currCosts.getWidth() && y < currCosts.getHeight())
            {
                const float prev1_t = prev1(x,y);
                const float prev2_t = prev2(x,y);
                const float currCosts_t = currCosts(x,y);


                // we only comput the parabola if we found a new minimal cost
                if (prev1_t <= bestPlaneCosts(x,y) && prev2_t > prev1(x,y) && currCosts_t > prev1(x,y))
                {
                    // parabola
                    const float denom = currCosts(x,y) + prev2(x,y) - 2*prev1(x,y);
                    float offset = 0.0f;
                    if (denom > 1e-5f)
                    {
                        offset = (prev2(x,y) - prev1(x,y))/denom - 0.5f;
                    }

                    bestPlanes(x,y) = prevPlaneIdx;
                    subPixelPlaneOffsets(x,y) = offset;
                    bestPlaneCosts(x,y) = prev1(x,y); // we use the actual cost of the minimal plane for robustnes
                }
            }

        }

        void planeSweepUpdateBestPlaneSubPixel(const DeviceBuffer<float>& currCosts, const DeviceBuffer<float>& prev1, const DeviceBuffer<float>& prev2,
                                               int width, int height, int prevPlaneIdx, DeviceBuffer<float>& bestPlaneCosts, DeviceBuffer<int>& bestPlanes, DeviceBuffer<float>& subPixelPlaneOffsets)
        {
            dim3 gridDim(getNumTiles(width, PLANE_SWEEP_TILE_WIDTH), getNumTiles(height, PLANE_SWEEP_TILE_HEIGHT));
            dim3 blockDim(PLANE_SWEEP_TILE_WIDTH, PLANE_SWEEP_TILE_HEIGHT);

            planeSweepUpdateBestPlaneSubPixelKernel<<<gridDim, blockDim>>>(currCosts, prev1, prev2, prevPlaneIdx, bestPlaneCosts, bestPlanes, subPixelPlaneOffsets);
            PSL_CUDA_CHECK_ERROR
        }

        __global__ void planeSweepUpdateBestAndSecondBestPlaneKernel(DeviceBuffer<float> newCosts, int currPlaneIndex, DeviceBuffer<float> bestPlaneCosts,
                                                                     DeviceBuffer<float> secondBestPlaneCosts, DeviceBuffer<int> bestPlanes)
        {
            // get position of outupt
            unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < newCosts.getWidth() && y < newCosts.getHeight())
            {

                if (newCosts(x,y) <= bestPlaneCosts(x,y))
                {
                    secondBestPlaneCosts(x,y) = bestPlaneCosts(x,y);
                    bestPlaneCosts(x,y) = newCosts(x,y);
                    bestPlanes(x,y) = currPlaneIndex;
                }
                else if(newCosts(x,y) <= secondBestPlaneCosts(x,y))
                {
                    secondBestPlaneCosts(x,y) = newCosts(x,y);
                }
            }

        }

        void planeSweepUpdateBestAndSecondBestPlane(const DeviceBuffer<float>& newCosts, int width, int height, int currPlaneIndex,
                                                    DeviceBuffer<float>& bestPlaneCosts, DeviceBuffer<float>& secondBestPlaneCosts, DeviceBuffer<int>& bestPlanes)
        {
            dim3 gridDim(getNumTiles(width, PLANE_SWEEP_TILE_WIDTH), getNumTiles(height, PLANE_SWEEP_TILE_HEIGHT));
            dim3 blockDim(PLANE_SWEEP_TILE_WIDTH, PLANE_SWEEP_TILE_HEIGHT);

            planeSweepUpdateBestAndSecondBestPlaneKernel<<<gridDim, blockDim>>>(newCosts, currPlaneIndex, bestPlaneCosts,
                                                                                secondBestPlaneCosts, bestPlanes);
            PSL_CUDA_CHECK_ERROR
        }

        __global__ void planeSweepUpdateBestAndSecondBestPlaneSubPixelKernel(DeviceBuffer<float> currCosts, DeviceBuffer<float> prev1, DeviceBuffer<float> prev2, int prevPlaneIdx,
                                                                             DeviceBuffer<float> bestPlaneCosts, DeviceBuffer<float> secondBestPlaneCosts, DeviceBuffer<int> bestPlanes, DeviceBuffer<float> subPixelPlaneOffsets)
        {
            // get position of outupt
            unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < currCosts.getWidth() && y < currCosts.getHeight())
            {
                const float prev1_t = prev1(x,y);
                const float prev2_t = prev2(x,y);
                const float currCosts_t = currCosts(x,y);

                // we only comput the parabola if we found a new minimal cost
                if (prev1_t <= bestPlaneCosts(x,y) && prev2_t > prev1_t && currCosts_t > prev1_t)
                {
                    // parabola
                    const float denom = currCosts(x,y) + prev2(x,y) - 2*prev1(x,y);
                    float offset = 0.0f;
                    if (denom > 1e-5f)
                    {
                        offset = (prev2(x,y) - prev1(x,y))/denom - 0.5f;
                    }

                    bestPlanes(x,y) = prevPlaneIdx;
                    subPixelPlaneOffsets(x,y) = offset;
                    bestPlaneCosts(x,y) = prev1(x,y); // we use the actual cost of the minimal plane for robustnes
                }
                else if (prev1(x,y) <= secondBestPlaneCosts(x,y))
                {
                    secondBestPlaneCosts(x,y) = prev1(x,y);
                }
            }
        }


        void planeSweepUpdateBestAndSecondBestPlaneSubPixel(const DeviceBuffer<float>& currCosts, const DeviceBuffer<float>& prev1, const DeviceBuffer<float>& prev2,
                                                            int width, int height, int prevPlaneIdx, DeviceBuffer<float>& bestPlaneCosts, DeviceBuffer<float>& secondBestPlaneCosts, DeviceBuffer<int>& bestPlanes, DeviceBuffer<float>& subPixelPlaneOffsets)

        {
            dim3 gridDim(getNumTiles(width, PLANE_SWEEP_TILE_WIDTH), getNumTiles(height, PLANE_SWEEP_TILE_HEIGHT));
            dim3 blockDim(PLANE_SWEEP_TILE_WIDTH, PLANE_SWEEP_TILE_HEIGHT);

            planeSweepUpdateBestAndSecondBestPlaneSubPixelKernel<<<gridDim, blockDim>>>(currCosts, prev1, prev2, prevPlaneIdx, bestPlaneCosts,
                                                                                        secondBestPlaneCosts, bestPlanes, subPixelPlaneOffsets);
            PSL_CUDA_CHECK_ERROR
        }


        __global__ void planeSweepMinFloatKernel(DeviceBuffer<float> buf1, DeviceBuffer<float> buf2, DeviceBuffer<float> bufMin)
        {
            // get position of outupt
            unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < buf1.getWidth() && y < buf1.getHeight())
            {
                const float cost1 = buf1(x,y);
                const float cost2 = buf2(x,y);

                bufMin(x,y) = min(cost1, cost2);
            }
        }

        void planeSweepMinFloat(DeviceBuffer<float>& buf1, DeviceBuffer<float>& buf2, DeviceBuffer<float>& minBuf)
        {
            dim3 gridDim(getNumTiles(buf1.getWidth(), PLANE_SWEEP_TILE_WIDTH), getNumTiles(buf1.getHeight(), PLANE_SWEEP_TILE_HEIGHT));
            dim3 blockDim(PLANE_SWEEP_TILE_WIDTH, PLANE_SWEEP_TILE_HEIGHT);

            planeSweepMinFloatKernel<<<gridDim, blockDim>>>(buf1, buf2, minBuf);
            PSL_CUDA_CHECK_ERROR
        }

        __global__ void uniqunessRatioKernel(DeviceBuffer<float> bestCost, DeviceBuffer<float> secondBestCost, DeviceBuffer<float> uniquenessRatios)
        {
            unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < uniquenessRatios.getWidth() && y < uniquenessRatios.getHeight())
            {
                if (bestCost(x,y) == 0)
                {
                    if (secondBestCost(x,y) > 0)
                    {
                        uniquenessRatios(x,y) = 0;
                    }
                    else
                    {
                        uniquenessRatios(x,y) = 1;
                    }
                }
                else
                {
                    uniquenessRatios(x,y) = bestCost(x,y)/secondBestCost(x,y);
                }
            }
        }

        void computeUniquenessRatio(DeviceBuffer<float>& bestCost, DeviceBuffer<float>& secondBestCost, DeviceBuffer<float>& uniquenessRatios)
        {
            dim3 gridDim(getNumTiles(bestCost.getWidth(), PLANE_SWEEP_TILE_WIDTH), getNumTiles(bestCost.getHeight(), PLANE_SWEEP_TILE_HEIGHT));
            dim3 blockDim(PLANE_SWEEP_TILE_WIDTH, PLANE_SWEEP_TILE_HEIGHT);

            uniqunessRatioKernel<<<gridDim, blockDim>>>(bestCost, secondBestCost, uniquenessRatios);
            PSL_CUDA_CHECK_ERROR
        }

        __global__ void planeSweepComputeBestDepthsKernel(DeviceBuffer<int> bestPlanes, float* planesAddr, size_t planesPitch,
                                                          float* bestDepthsDAddr, size_t bestDepthsDPitch, float3 KrefInvCol1, float3 KrefInvCol2, float3 KrefInvCol3, float XiRef)
        {
            unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < bestPlanes.getWidth() && y < bestPlanes.getHeight())
            {
                const int planeIdx = bestPlanes(x,y);

                // apply Krefinv
                float mx = KrefInvCol1.x*x + KrefInvCol3.x;
                float my = KrefInvCol2.y*y + KrefInvCol3.y;

                // apply h^-1 of camera model
                float mxPmySqr = mx*mx + my*my;
                float D = 1 + (1 - XiRef*XiRef)*mxPmySqr;

                if (D < 0)
                    D = -D;

                float fact = (XiRef + sqrtf(D))/(mxPmySqr + 1);

                float xx = fact*mx;
                float yy = fact*my;
                float zz = fact - XiRef;

                xx /= zz;
                yy /= zz;

                float3 planeN;
                planeN.x = planesAddr[planeIdx];
                planeN.y = *((float*)((char*)planesAddr + planesPitch) + planeIdx);
                planeN.z = *((float*)((char*)planesAddr + 2*planesPitch) + planeIdx);

                const float planeD = *((float*)((char*)planesAddr + 3*planesPitch) + planeIdx);

                const float denom = xx*planeN.x + yy*planeN.y + planeN.z;

                *((float*)((char*)bestDepthsDAddr + y*bestDepthsDPitch) + x) = -planeD/denom;
            }
        }


        void planeSweepComputeBestDepths(DeviceBuffer<int>& bestPlanes, int numPlanes, std::vector<float>& planes,
                                         float* bestDepthsAddr, int bestDepthsPitch, std::vector<float> &KrefInv, float XiRef)
        {
            const int width = bestPlanes.getWidth();
            const int height = bestPlanes.getHeight();

            // allocate device memory for planes
            float* planesAddr;
            size_t planesPitch;
            PSL_CUDA_CHECKED_CALL( cudaMallocPitch(&planesAddr, &planesPitch, sizeof(float)*numPlanes, 4); )
            // copy planes to device
            PSL_CUDA_CHECKED_CALL( cudaMemcpy2D(planesAddr, planesPitch, &(planes[0]), sizeof(float)*numPlanes, sizeof(float)*numPlanes, 4, cudaMemcpyHostToDevice); )

            // allocate memory on device for best depths
            float* bestDepthsDAddr;
            size_t bestDepthsDPitch;
            PSL_CUDA_CHECKED_CALL( cudaMallocPitch(&bestDepthsDAddr, &bestDepthsDPitch, sizeof(float)*width, height); )

            float3 KrefInvCol1;
            KrefInvCol1.x = KrefInv[0];
            KrefInvCol1.y = KrefInv[3];
            KrefInvCol1.z = KrefInv[6];
            float3 KrefInvCol2;
            KrefInvCol2.x = KrefInv[1];
            KrefInvCol2.y = KrefInv[4];
            KrefInvCol2.z = KrefInv[7];
            float3 KrefInvCol3;
            KrefInvCol3.x = KrefInv[2];
            KrefInvCol3.y = KrefInv[5];
            KrefInvCol3.z = KrefInv[8];

            dim3 gridDim(getNumTiles(width, PLANE_SWEEP_TILE_WIDTH), getNumTiles(height, PLANE_SWEEP_TILE_HEIGHT));
            dim3 blockDim(PLANE_SWEEP_TILE_WIDTH, PLANE_SWEEP_TILE_HEIGHT);

            planeSweepComputeBestDepthsKernel<<<gridDim, blockDim>>>(bestPlanes, planesAddr, planesPitch,
                                                                     bestDepthsDAddr, bestDepthsDPitch, KrefInvCol1, KrefInvCol2, KrefInvCol3, XiRef);
            PSL_CUDA_CHECK_ERROR

            // download result from gpu
            PSL_CUDA_CHECKED_CALL( cudaMemcpy2D(bestDepthsAddr, bestDepthsPitch, bestDepthsDAddr, bestDepthsDPitch, sizeof(float)*width, height, cudaMemcpyDeviceToHost); )

            // free device memory
            PSL_CUDA_CHECKED_CALL( cudaFree(planesAddr); )
            PSL_CUDA_CHECKED_CALL( cudaFree(bestDepthsDAddr); )

        }


        __global__ void planeSweepComputeBestDepthsSubPixelInverseKernel(DeviceBuffer<int> bestPlanes, DeviceBuffer<float> subPixelPlaneOffsets, float* planesAddr, size_t planesPitch,
                                                                  int numPlanes, float* bestDepthsDAddr, size_t bestDepthsDPitch, float3 KrefInvCol1, float3 KrefInvCol2, float3 KrefInvCol3, float XiRef)
        {
            unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < bestPlanes.getWidth() && y < bestPlanes.getHeight())
            {
                const int planeIdx = bestPlanes(x,y);

                // apply Krefinv
                float mx = KrefInvCol1.x*x + KrefInvCol3.x;
                float my = KrefInvCol2.y*y + KrefInvCol3.y;

                // apply h^-1 of camera model
                float mxPmySqr = mx*mx + my*my;
                float D = 1 + (1 - XiRef*XiRef)*mxPmySqr;

                if (D < 0)
                    D = -D;

                float fact = (XiRef + sqrtf(D))/(mxPmySqr + 1);

                float xx = fact*mx;
                float yy = fact*my;
                float zz = fact - XiRef;

                xx /= zz;
                yy /= zz;

                float3 planeN;
                planeN.x = planesAddr[planeIdx];
                planeN.y = *((float*)((char*)planesAddr + planesPitch) + planeIdx);
                planeN.z = *((float*)((char*)planesAddr + 2*planesPitch) + planeIdx);

                float planeD = *((float*)((char*)planesAddr + 3*planesPitch) + planeIdx);

                if (subPixelPlaneOffsets(x,y) < 0)
                {
                    const int oPlaneIdx = planeIdx - 1;

                    if (oPlaneIdx >= 0)
                    {
                        float oPlaneD = *((float*)((char*)planesAddr + 3*planesPitch) + oPlaneIdx);

                        float dStep = (1.0f/oPlaneD - 1.0f/planeD)*subPixelPlaneOffsets(x,y);
                        planeD = 1.0f/(1.0f/planeD - dStep);
                    }

                }
                if (subPixelPlaneOffsets(x,y) > 0)
                {
                    const int oPlaneIdx = planeIdx + 1;

                    if (oPlaneIdx < numPlanes)
                    {
                        float oPlaneD = *((float*)((char*)planesAddr + 3*planesPitch) + oPlaneIdx);

                        float dStep = (1.0f/planeD - 1.0f/oPlaneD)*subPixelPlaneOffsets(x,y);
                        planeD = 1.0f/(1.0f/planeD - dStep);
                    }
                }

                const float denom = xx*planeN.x + yy*planeN.y + planeN.z;

                *((float*)((char*)bestDepthsDAddr + y*bestDepthsDPitch) + x) = -planeD/denom;
            }
        }


        void planeSweepComputeBestDepthsSubPixelInverse(DeviceBuffer<int>& bestPlanes, DeviceBuffer<float>& subPixelPlaneOffsets, int numPlanes, std::vector<float>& planes,
                                         float* bestDepthsAddr, int bestDepthsPitch, std::vector<float> &KrefInv, float XiRef)
        {
            const int width = bestPlanes.getWidth();
            const int height = bestPlanes.getHeight();

            // allocate device memory for planes
            float* planesAddr;
            size_t planesPitch;
            PSL_CUDA_CHECKED_CALL( cudaMallocPitch(&planesAddr, &planesPitch, sizeof(float)*numPlanes, 4); )
            // copy planes to device
            PSL_CUDA_CHECKED_CALL( cudaMemcpy2D(planesAddr, planesPitch, &(planes[0]), sizeof(float)*numPlanes, sizeof(float)*numPlanes, 4, cudaMemcpyHostToDevice); )

            // allocate memory on device for best depths
            float* bestDepthsDAddr;
            size_t bestDepthsDPitch;
            PSL_CUDA_CHECKED_CALL( cudaMallocPitch(&bestDepthsDAddr, &bestDepthsDPitch, sizeof(float)*width, height); )

            float3 KrefInvCol1;
            KrefInvCol1.x = KrefInv[0];
            KrefInvCol1.y = KrefInv[3];
            KrefInvCol1.z = KrefInv[6];
            float3 KrefInvCol2;
            KrefInvCol2.x = KrefInv[1];
            KrefInvCol2.y = KrefInv[4];
            KrefInvCol2.z = KrefInv[7];
            float3 KrefInvCol3;
            KrefInvCol3.x = KrefInv[2];
            KrefInvCol3.y = KrefInv[5];
            KrefInvCol3.z = KrefInv[8];

            dim3 gridDim(getNumTiles(width, PLANE_SWEEP_TILE_WIDTH), getNumTiles(height, PLANE_SWEEP_TILE_HEIGHT));
            dim3 blockDim(PLANE_SWEEP_TILE_WIDTH, PLANE_SWEEP_TILE_HEIGHT);

            planeSweepComputeBestDepthsSubPixelInverseKernel<<<gridDim, blockDim>>>(bestPlanes, subPixelPlaneOffsets, planesAddr, planesPitch, numPlanes,
                                                                     bestDepthsDAddr, bestDepthsDPitch, KrefInvCol1, KrefInvCol2, KrefInvCol3, XiRef);
            PSL_CUDA_CHECK_ERROR

            // download result from gpu
            PSL_CUDA_CHECKED_CALL( cudaMemcpy2D(bestDepthsAddr, bestDepthsPitch, bestDepthsDAddr, bestDepthsDPitch, sizeof(float)*width, height, cudaMemcpyDeviceToHost); )

            // free device memory
            PSL_CUDA_CHECKED_CALL( cudaFree(planesAddr); )
            PSL_CUDA_CHECKED_CALL( cudaFree(bestDepthsDAddr); )
        }


        __global__ void planeSweepComputeBestDepthsSubPixelDirectKernel(DeviceBuffer<int> bestPlanes, DeviceBuffer<float> subPixelPlaneOffsets, float* planesAddr, size_t planesPitch,
                                                                  int numPlanes, float* bestDepthsDAddr, size_t bestDepthsDPitch, float3 KrefInvCol1, float3 KrefInvCol2, float3 KrefInvCol3, float XiRef)
        {
            unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < bestPlanes.getWidth() && y < bestPlanes.getHeight())
            {
                const int planeIdx = bestPlanes(x,y);

                // apply Krefinv
                float mx = KrefInvCol1.x*x + KrefInvCol3.x;
                float my = KrefInvCol2.y*y + KrefInvCol3.y;

                // apply h^-1 of camera model
                float mxPmySqr = mx*mx + my*my;
                float D = 1 + (1 - XiRef*XiRef)*mxPmySqr;

                if (D < 0)
                    D = -D;

                float fact = (XiRef + sqrtf(D))/(mxPmySqr + 1);

                float xx = fact*mx;
                float yy = fact*my;
                float zz = fact - XiRef;

                xx /= zz;
                yy /= zz;

                float3 planeN;
                planeN.x = planesAddr[planeIdx];
                planeN.y = *((float*)((char*)planesAddr + planesPitch) + planeIdx);
                planeN.z = *((float*)((char*)planesAddr + 2*planesPitch) + planeIdx);

                float planeD = *((float*)((char*)planesAddr + 3*planesPitch) + planeIdx);

                if (subPixelPlaneOffsets(x,y) < 0)
                {
                    const int oPlaneIdx = planeIdx - 1;

                    if (oPlaneIdx >= 0)
                    {
                        float oPlaneD = *((float*)((char*)planesAddr + 3*planesPitch) + oPlaneIdx);

                        float dStep = (planeD - oPlaneD)*subPixelPlaneOffsets(x,y);
                        planeD = planeD + dStep;
                    }

                }
                if (subPixelPlaneOffsets(x,y) > 0)
                {
                    const int oPlaneIdx = planeIdx + 1;


                    if (oPlaneIdx < numPlanes)
                    {
                        float oPlaneD = *((float*)((char*)planesAddr + 3*planesPitch) + oPlaneIdx);

                        float dStep = (oPlaneD - planeD)*subPixelPlaneOffsets(x,y);
                        planeD = planeD + dStep;
                    }
                }

                const float denom = xx*planeN.x + yy*planeN.y + planeN.z;

                *((float*)((char*)bestDepthsDAddr + y*bestDepthsDPitch) + x) = -planeD/denom;
            }
        }


        void planeSweepComputeBestDepthsSubPixelDirect(DeviceBuffer<int>& bestPlanes, DeviceBuffer<float>& subPixelPlaneOffsets, int numPlanes, std::vector<float>& planes,
                                                       float* bestDepthsAddr, int bestDepthsPitch, std::vector<float> &KrefInv, float XiRef)
        {
            const int width = bestPlanes.getWidth();
            const int height = bestPlanes.getHeight();

            // allocate device memory for planes
            float* planesAddr;
            size_t planesPitch;
            PSL_CUDA_CHECKED_CALL( cudaMallocPitch(&planesAddr, &planesPitch, sizeof(float)*numPlanes, 4); )
            // copy planes to device
            PSL_CUDA_CHECKED_CALL( cudaMemcpy2D(planesAddr, planesPitch, &(planes[0]), sizeof(float)*numPlanes, sizeof(float)*numPlanes, 4, cudaMemcpyHostToDevice); )

            // allocate memory on device for best depths
            float* bestDepthsDAddr;
            size_t bestDepthsDPitch;
            PSL_CUDA_CHECKED_CALL( cudaMallocPitch(&bestDepthsDAddr, &bestDepthsDPitch, sizeof(float)*width, height); )

            float3 KrefInvCol1;
            KrefInvCol1.x = KrefInv[0];
            KrefInvCol1.y = KrefInv[3];
            KrefInvCol1.z = KrefInv[6];
            float3 KrefInvCol2;
            KrefInvCol2.x = KrefInv[1];
            KrefInvCol2.y = KrefInv[4];
            KrefInvCol2.z = KrefInv[7];
            float3 KrefInvCol3;
            KrefInvCol3.x = KrefInv[2];
            KrefInvCol3.y = KrefInv[5];
            KrefInvCol3.z = KrefInv[8];

            dim3 gridDim(getNumTiles(width, PLANE_SWEEP_TILE_WIDTH), getNumTiles(height, PLANE_SWEEP_TILE_HEIGHT));
            dim3 blockDim(PLANE_SWEEP_TILE_WIDTH, PLANE_SWEEP_TILE_HEIGHT);

            planeSweepComputeBestDepthsSubPixelDirectKernel<<<gridDim, blockDim>>>(bestPlanes, subPixelPlaneOffsets, planesAddr, planesPitch, numPlanes,
                                                                     bestDepthsDAddr, bestDepthsDPitch, KrefInvCol1, KrefInvCol2, KrefInvCol3, XiRef);
            PSL_CUDA_CHECK_ERROR

            // download result from gpu
            PSL_CUDA_CHECKED_CALL( cudaMemcpy2D(bestDepthsAddr, bestDepthsPitch, bestDepthsDAddr, bestDepthsDPitch, sizeof(float)*width, height, cudaMemcpyDeviceToHost); )

            // free device memory
            PSL_CUDA_CHECKED_CALL( cudaFree(planesAddr); )
            PSL_CUDA_CHECKED_CALL( cudaFree(bestDepthsDAddr); )

        }

        __forceinline__ __device__ float computeWarpedGrayscaleTexturePixel(int x , int y, int width, int height,
                                                                            float krefinv11, float krefinv13, float krefinv22, float krefinv23, float xiref,
                                                                            float kother11, float kother13, float kother22, float kother23, float xiother,
                                                                            float rt11, float rt12, float rt13, float rt21, float rt22, float rt23, float rt31, float rt32, float rt33)
        {
            //apply Krefinv
            const float mx = krefinv11*x + krefinv13;
            const float my = krefinv22*y + krefinv23;

            // apply h^-1 of camera model
            const float mxPmySqr = mx*mx + my*my;

            const float fact = (xiref + sqrtf(abs(1 + (1 - xiref*xiref)*mxPmySqr)))/(mxPmySqr + 1);

            const float xx = fact*mx;
            const float yy = fact*my;
            const float zz = fact - xiref;

            // apply rt
            const float xxw = rt11*xx + rt12*yy + rt13*zz;
            const float yyw = rt21*xx + rt22*yy + rt23*zz;
            const float zzw = rt31*xx + rt32*yy + rt33*zz;

            // apply h
            const float lengthInv = 1.0f/sqrtf(xxw*xxw + yyw*yyw + zzw*zzw);
            const float zzzwInv = 1.0f/(zzw*lengthInv + xiother);

            // apply Kother

            const float xw = kother11*xxw*lengthInv*zzzwInv + kother13;
            const float yw = kother22*yyw*lengthInv*zzzwInv + kother23;

            const float u = (xw+0.5f) / (float) width;
            const float v = (yw+0.5f) / (float) height;

            const float1 pix = tex2D(planeSweepGrayscaleTexture, u, v);

            return pix.x*255;
        }

        __forceinline__ __device__ float computeZNCC(float normalizer, float ref, float refSqr, float other, float otherSqr, float prod)
        {

            const float numerator = prod - normalizer*(ref*other);

            float denomRef = refSqr - normalizer*ref*ref;
            float denomOther = otherSqr - normalizer*other*other;

            if (denomRef < 2)
                denomRef = 2;

            if (denomOther < 2)
                denomOther = 2;

            return (1 - (numerator/(sqrtf(denomRef)*sqrtf(denomOther))))/2.0f;
        }
    }
}
