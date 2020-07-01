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

#include <psl/deviceImage.h>
#include <psl/deviceImage.cuh>

namespace PSL_CUDA
{
    namespace CudaFishEyeImageProcessorDeviceCode
    {
        texture<uchar1, 2, cudaReadModeNormalizedFloat> fishEyeImageProcessorGrayscaleTexture;
        cudaChannelFormatDesc fishEyeImageProcessorGrayscaleChannelDesc;

        bool fishEyeImageProcessorTexturingInitialized = false;


        void fishEyeImageProcessorInitTexturing()
        {
            if (!fishEyeImageProcessorTexturingInitialized)
            {
                // textures for image processing
                fishEyeImageProcessorGrayscaleChannelDesc = cudaCreateChannelDesc(8,0,0,0,cudaChannelFormatKindUnsigned);

                fishEyeImageProcessorGrayscaleTexture.addressMode[0] = cudaAddressModeWrap;
                fishEyeImageProcessorGrayscaleTexture.addressMode[1] = cudaAddressModeWrap;
                fishEyeImageProcessorGrayscaleTexture.filterMode = cudaFilterModeLinear;
                fishEyeImageProcessorGrayscaleTexture.normalized = true;

                fishEyeImageProcessorTexturingInitialized = true;
            }
        }

        __global__ void fishEyeImageProcessorUndistortKernel(DeviceImage outputImg, float k1_input, float k2_input, float p1_input, float p2_input,
                                                             float k11_input, float k13_input, float k22_input, float k23_input,
                                                             float k11inv_output, float k13inv_output, float k22inv_output, float k23inv_output,
                                                             int inputImgWidth, int inputImgHeight)
        {
            // get position of outupt
            const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < outputImg.getWidth() && y < outputImg.getHeight())
            {
                // compute position
                // apply Kinv
                float mx = k11inv_output*x + k13inv_output;
                float my = k22inv_output*y + k23inv_output;

                // apply distortion
                float rho_sqr =  mx*mx + my*my;

                float ratio_radial = k1_input*rho_sqr + k2_input*rho_sqr*rho_sqr;

                float du_radial_x = mx * ratio_radial;
                float du_radial_y = my * ratio_radial;

                float du_tangent_x = 2.0f*p1_input*mx*my + p2_input*(rho_sqr + 2.0f * mx*mx);
                float du_tangent_y = p1_input*(rho_sqr + 2.0f*my*my) + 2.0f*p2_input*mx*my;

                float udx = mx + du_radial_x + du_tangent_x;
                float udy = my + du_radial_y + du_tangent_y;

                float xd = k11_input*udx + k13_input;
                float yd = k22_input*udy + k23_input;

                const float u = (xd+0.5f) / (float) inputImgWidth;
                const float v = (yd+0.5f) / (float) inputImgHeight;

                const float1 pix = tex2D(fishEyeImageProcessorGrayscaleTexture, u, v);

                outputImg(x,y) = pix.x*255;
            }
        }

        void fishEyeImageProcessorUndistort(DeviceImage& inputImg, DeviceImage& outputImg, double k1_input,
                                            double k2_input, double p1_input, double p2_input,
                                            double k11_input, double k13_input, double k22_input, double k23_input,
                                            double k11inv_output, double k13inv_output, double k22inv_output, double k23inv_output)
        {
            // bind texture
            PSL_CUDA_CHECKED_CALL( cudaBindTexture2D(0, fishEyeImageProcessorGrayscaleTexture, inputImg.getAddr(), fishEyeImageProcessorGrayscaleChannelDesc, inputImg.getWidth(), inputImg.getHeight(), inputImg.getPitch()); )

            dim3 gridDim(getNumTiles(outputImg.getWidth(), TILE_WIDTH), getNumTiles(outputImg.getHeight(), TILE_HEIGHT));
            dim3 blockDim(TILE_WIDTH, TILE_HEIGHT);


            fishEyeImageProcessorUndistortKernel<<<gridDim, blockDim>>>(outputImg, (float) k1_input, (float) k2_input, (float) p1_input, (float) p2_input,
                                                                        (float) k11_input, (float) k13_input, (float) k22_input, (float) k23_input, (float) k11inv_output, (float) k13inv_output, (float) k22inv_output, (float) k23inv_output,
                                                                        inputImg.getWidth(), inputImg.getHeight());
            PSL_CUDA_CHECK_ERROR

            PSL_CUDA_CHECKED_CALL( cudaUnbindTexture(fishEyeImageProcessorGrayscaleTexture); )

        }

        __global__ void fishEyeImageProcessorUndistortRectifyKernel(DeviceImage outputImg, float xi_input, float k1_input, float k2_input, float p1_input, float p2_input,
                                                                    float k11_input, float k13_input, float k22_input, float k23_input,
                                                                    float k11inv_output, float k13inv_output, float k22inv_output, float k23inv_output,
                                                                    int inputImgWidth, int inputImgHeight)
        {
            // get position of outupt
            const unsigned int x = blockIdx.x*blockDim.x + threadIdx.x;
            const unsigned int y = blockIdx.y*blockDim.y + threadIdx.y;

            if (x < outputImg.getWidth() && y < outputImg.getHeight())
            {
                // compute position
                // apply Kinv
                float mx = k11inv_output*x + k13inv_output;
                float my = k22inv_output*y + k23inv_output;

                float lengthInv = 1.0f/sqrtf(mx*mx + my*my + 1.0f);

                float ux = mx*lengthInv;
                float uy = my*lengthInv;
                float uz = lengthInv + xi_input;
                ux /= uz;
                uy /= uz;

                // apply distortion
                float rho_sqr =  ux*ux + uy*uy;

                float ratio_radial = k1_input*rho_sqr + k2_input*rho_sqr*rho_sqr;

                float du_radial_x = ux * ratio_radial;
                float du_radial_y = uy * ratio_radial;

                float du_tangent_x = 2.0f*p1_input*ux*uy + p2_input*(rho_sqr + 2.0f * ux*ux);
                float du_tangent_y = p1_input*(rho_sqr + 2.0f*uy*uy) + 2.0f*p2_input*ux*uy;

                float udx = ux + du_radial_x + du_tangent_x;
                float udy = uy + du_radial_y + du_tangent_y;

                float xd = k11_input*udx + k13_input;
                float yd = k22_input*udy + k23_input;

                const float u = (xd+0.5f) / (float) inputImgWidth;
                const float v = (yd+0.5f) / (float) inputImgHeight;

                const float1 pix = tex2D(fishEyeImageProcessorGrayscaleTexture, u, v);

                outputImg(x,y) = pix.x*255;
            }
        }

        void fishEyeImageProcessorUndistortRectify(DeviceImage& inputImg, DeviceImage& outputImg, double xi_input, double k1_input,
                                                   double k2_input, double p1_input, double p2_input,
                                                   double k11_input, double k13_input, double k22_input, double k23_input,
                                                   double k11inv_output, double k13inv_output, double k22inv_output, double k23inv_output)
        {
            // bind texture
            PSL_CUDA_CHECKED_CALL( cudaBindTexture2D(0, fishEyeImageProcessorGrayscaleTexture, inputImg.getAddr(), fishEyeImageProcessorGrayscaleChannelDesc, inputImg.getWidth(), inputImg.getHeight(), inputImg.getPitch()); )

            dim3 gridDim(getNumTiles(outputImg.getWidth(), TILE_WIDTH), getNumTiles(outputImg.getHeight(), TILE_HEIGHT));
            dim3 blockDim(TILE_WIDTH, TILE_HEIGHT);


            fishEyeImageProcessorUndistortRectifyKernel<<<gridDim, blockDim>>>(outputImg, (float) xi_input, (float) k1_input, (float) k2_input, (float) p1_input, (float) p2_input,
                                                                               (float) k11_input, (float) k13_input, (float) k22_input, (float) k23_input, (float) k11inv_output, (float) k13inv_output, (float) k22inv_output, (float) k23inv_output,
                                                                               inputImg.getWidth(), inputImg.getHeight());
            PSL_CUDA_CHECK_ERROR

            PSL_CUDA_CHECKED_CALL( cudaUnbindTexture(fishEyeImageProcessorGrayscaleTexture); )
        }
    }
}
