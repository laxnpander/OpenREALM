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

#include <iostream>
#include <vector>
#include <string>
#include <sstream>


#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <psl/exception.h>
#include <psl/cudaPlaneSweep.h>
#include <psl/cudaCommon.h>
#include <psl/deviceImage.h>


using std::cerr;
using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::stringstream;

using namespace cv;

using namespace PSL;
using namespace PSL_CUDA::CudaPlaneSweepDeviceCode;
using namespace PSL_CUDA;

CudaPlaneSweep::CudaPlaneSweep()
{

    nextId = 0;

    // set invalid default values
    nearZ = -1;
    farZ = -1;
    matchWindowHeight = -1;
    matchWindowWidth = -1;
    matchWindowRadiusX = -1;
    matchWindowRadiusY = -1;

    // set default parameters
    occlusionMode = PLANE_SWEEP_OCCLUSION_NONE;
    occlusionBestK = 1;

    scale = 1;

    planeGenerationMode = PLANE_SWEEP_PLANEMODE_UNIFORM_DEPTH;

    outputBestDepthEnabled = true;
    outputBestCostsEnabled = false;
    colorMatchingEnabled = false;
    outputCostVolumeEnabled = false;
    outputUniquenessRatioEnabled = false;
    subPixelEnabled = true;

    subPixelInterpMode = PLANE_SWEEP_SUB_PIXEL_INTERP_INVERSE;

    boxFilterBeforeOcclusion = true;

    matchingCosts = PLANE_SWEEP_SAD;

    // init cuda textures
    planeSweepInitTexturing();
}

CudaPlaneSweep::~CudaPlaneSweep()
{
    // delete all the images from the gpu
    for (map<int, CudaPlaneSweepImage>::iterator it = images.begin(); it != images.end(); it++)
    {
        it->second.devImg.deallocate();
    }

    deallocateBuffers();
}

int CudaPlaneSweep::addImage(Mat image, CameraMatrix<double>& cam)
{
    CudaPlaneSweepImage cPSI;
	cPSI.cam = cam;

    if (scale != 1)
    {
        // scale image
        Mat scaledImage;
        if (scale < 1)
            resize(image, scaledImage, Size(0,0), scale, scale, INTER_AREA);
        else
            resize(image, scaledImage, Size(0,0) , scale, scale, INTER_LINEAR);

        image = scaledImage;

//        cv::imshow("image", image);
//        cv::waitKey(500);

        // scale cam
        cPSI.cam.scaleK(scale, scale);
    }

    if (colorMatchingEnabled && image.channels() == 4)
    {
        // copy rgba data
        cPSI.devImg.allocatePitchedAndUpload(image);
    }
    else if (colorMatchingEnabled && image.channels() == 3)
    {
        // add alpha channel first
        Mat corrImg;
        cvtColor(image, corrImg, CV_BGR2BGRA);
        cPSI.devImg.allocatePitchedAndUpload(corrImg);
    }
    else if (!colorMatchingEnabled && image.channels() == 3)
    {
        // convert to grayscale
        Mat corrImg;
        cvtColor(image, corrImg, CV_BGR2GRAY);
        cPSI.devImg.allocatePitchedAndUpload(corrImg);
    }
    else if(!colorMatchingEnabled && image.channels() == 1)
    {
        // copy inensity data
        cPSI.devImg.allocatePitchedAndUpload(image);
    }
    else
    {
        PSL_THROW_EXCEPTION("Image format not supported for current color mode.");
    }

    int id = nextId++;

    images[id] = cPSI;

    return id;
}

int CudaPlaneSweep::addDeviceImage(DeviceImage& devImage, CameraMatrix<double>& cam)
{
    CudaPlaneSweepImage cPSI;
    cPSI.devImg = devImage;
    cPSI.cam = cam;

    int id = nextId++;

    images[id] = cPSI;

    return id;
}

void CudaPlaneSweep::deleteImage(int id)
{

    CudaPlaneSweepImage img;

    if (images.count(id) == 1)
    {
        img = images[id];
    }
    else
    {
        stringstream strstream;
        strstream << "Image with ID " << id << " does not exist.";
        PSL_THROW_EXCEPTION(strstream.str().c_str());
    }

    // delete image on the device
    img.devImg.deallocate();

    // delete image form the map
    images.erase(id);
}

void CudaPlaneSweep::setZRange(double nearZ, double farZ)
{
    this->nearZ = nearZ;
    this->farZ = farZ;
}

void CudaPlaneSweep::setMatchWindowSize(int w, int h)
{
    this->matchWindowHeight = h;
    this->matchWindowWidth = w;

    this->matchWindowRadiusX = w/2;
    this->matchWindowRadiusY = h/2;
}

void CudaPlaneSweep::setNumPlanes(int num)
{
    this->numPlanes = num;
}

void CudaPlaneSweep::setOcclusionMode(PlaneSweepOcclusionMode occlusionMode)
{
    this->occlusionMode = occlusionMode;
}

void CudaPlaneSweep::setOcclusionBestK(int bestK)
{
    occlusionBestK = bestK;
}

double CudaPlaneSweep::largestBaseline(const CudaPlaneSweepImage& refImg)
{

    double lBaseline = 0;
    Matrix<double, 3, 1> refC = refImg.cam.getC();

    for (map<int, CudaPlaneSweepImage>::iterator it = images.begin(); it != images.end(); it++)
    {
        if (refImg.devImg.getAddr() == it->second.devImg.getAddr())
        {
            continue;
        }

        // compute baseline
        Matrix<double, 3, 1> thisC = it->second.cam.getC();
        double baseline = (refC-thisC).norm();
        if (baseline > lBaseline)
            lBaseline = baseline;
    }

    return lBaseline;
}

void CudaPlaneSweep::process(int refImgId, Grid<Vector4d> &planes)
{
    int numPlanes = planes.getWidth();
    //this->planes = planes;

    // check if values are set
    if (!(matchWindowHeight > 0 && matchWindowWidth > 0 &&
          numPlanes > 0))
    {
        PSL_THROW_EXCEPTION("Parameters not set properly")
    }

    CudaPlaneSweepImage refImg;

    if (images.count(refImgId) == 1)
    {
        refImg = images[refImgId] ;
    }
    else
    {
        stringstream strstream;
        strstream << "Image with ID " << refImgId << " does not exist.";
        PSL_THROW_EXCEPTION(strstream.str().c_str());
    }

    refImgCam = refImg.cam;

    if (outputCostVolumeEnabled)
    {
        costVolume = Grid<float>(refImg.devImg.getWidth(), refImg.devImg.getHeight(), numPlanes);
    }


    // allocate the warping buffer on the gpu
//    planeSweepCreateImage(refImg.w, refImg.h, colorMatchingEnabled, this->warpingBuffer.imgAddr, this->warpingBuffer.pitch);

    // allocate the costs buffer on the gpu
//    planeSweepCreateCostBuffer(refImg.w, refImg.h, this->costBuffer.addr, this->costBuffer.pitch);

    // allocate a cost buffer to store the results of the first box filter pass
    boxFilterTempBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());

    // allocate the cost accum buffer on the gpu
    costAccumBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
//    planeSweepCreateFloatBuffer(refImg.w, refImg.h, this->costAccumBuffer.addr, this->costAccumBuffer.pitch);

    if (subPixelEnabled)
    {
        subPixelCostAccumBufferPrev1.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        subPixelCostAccumBufferPrev2.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        subPixelPlaneOffsetsBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        subPixelPlaneOffsetsBuffer.clear(0.0f);
    }

    if (outputUniquenessRatioEnabled)
    {
        secondBestPlaneCostBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        secondBestPlaneCostBuffer.clear(1e6);
    }


    if (outputBestDepthEnabled || outputUniquenessRatioEnabled)
    {
        // allocate best plane buffer
        bestPlaneBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        // initialize to zero,
        // this is necessary because the buffer on the gpu can be bigger than the actual image and we need to have valid plane indices everywhere
        // for the best depth computation
        bestPlaneBuffer.clear(0);

        // allocate best plane cost buffer
        bestPlaneCostBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        // initialize to big value
        bestPlaneCostBuffer.clear(1e6);
    }

    if (occlusionMode == PLANE_SWEEP_OCCLUSION_BEST_K)
    {
        // allocate buffers for the matching costs
        costBuffers.resize(images.size()-1);
        for (unsigned int i = 0; i < costBuffers.size(); i++)
        {
            costBuffers[i].reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        }

        // allocate buffers to find the best K
        bestKBuffer0.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        bestKBuffer1.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
    }
    else if (occlusionMode == PLANE_SWEEP_OCCLUSION_REF_SPLIT)
    {
        // allocate buffers for accumulated matching costs
        costAccumBeforeBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        costAccumAfterBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
    }

    double accumScale0 = 0, accumScale1 = 0;

    if (occlusionMode == PLANE_SWEEP_OCCLUSION_NONE)
    {
        accumScale0 = (double)1 / (double) (images.size()-1);
    }
    else if (occlusionMode == PLANE_SWEEP_OCCLUSION_REF_SPLIT)
    {
        int numBefore = 0;
        int numAfter = 0;
        for(map<int, CudaPlaneSweepImage>::iterator it = images.begin(); it != images.end(); it++)
        {
            if (it->first < refImgId)
            {
                numBefore++;
            }
            else if (it->first > refImgId)
            {
                numAfter++;
            }
        }

        if (numBefore == 0 || numAfter == 0)
        {
            PSL_THROW_EXCEPTION( "Reference Image cannot be at the border of the sequence with PLANE_SWEEP_OCCLUSION_REF_SPLIT" )
        }

        accumScale0 = (double) 1 / (double) numBefore;
        accumScale1 = (double) 1 / (double) numAfter;
    }
    else if (occlusionMode == PLANE_SWEEP_OCCLUSION_BEST_K)
    {
        accumScale0 = (double) 1 / (double) occlusionBestK;
    }


    if (matchingCosts == PLANE_SWEEP_ZNCC)
    {
        // the matching cost is NCC so we need to allocate some more buffers
        refImgBoxFilterBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        otherImgBoxFilterBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        refImgSqrBoxFilterBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        otherImgSqrBoxFilterBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        imgProdBoxFilterBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        boxFilterSqrTempBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());
        boxFilterProdTempBuffer.reallocatePitched(refImg.devImg.getWidth(), refImg.devImg.getHeight());

        // the box filtering only dependant on the reference image can be done only once before entering the main loop
        planeSweepBoxFilterImageAndSqrImage(refImg.devImg, refImgBoxFilterBuffer, refImgSqrBoxFilterBuffer,
                                            matchWindowRadiusX, matchWindowRadiusY);
    }


    for (int i = 0; i < numPlanes; i++)
    {

        if (occlusionMode == PLANE_SWEEP_OCCLUSION_NONE)
        {
            costAccumBuffer.clear(0);
        }
        else if (occlusionMode == PLANE_SWEEP_OCCLUSION_REF_SPLIT)
        {
            costAccumBeforeBuffer.clear(0);
            costAccumAfterBuffer.clear(0);
        }


        int j = 0;
        for (map<int, CudaPlaneSweepImage>::iterator it = images.begin(); it != images.end(); it++)
        {
            if (it->first == refImgId)
                continue;

            // match the image
            switch(occlusionMode)
            {
            case PLANE_SWEEP_OCCLUSION_NONE:
            {
                float H[9];
                planeHomography(planes(i,0), refImg.cam, it->second.cam, H);

                switch(matchingCosts)
                {
                case PLANE_SWEEP_SAD:
                    planeSweepWarpADAccum(it->second.devImg, colorMatchingEnabled, H,
                                      refImg.devImg, (float) accumScale0, costAccumBuffer);
                    break;
                case PLANE_SWEEP_ZNCC:
                    planeSweepWarpZNCCAccum(it->second.devImg, H, refImg.devImg,
                                           refImgBoxFilterBuffer, refImgSqrBoxFilterBuffer,
                                           (float) accumScale0, costAccumBuffer, matchWindowRadiusX, matchWindowRadiusY);
                    break;
                }
                break;
            }
            case PLANE_SWEEP_OCCLUSION_BEST_K:
            {
                    matchImage(refImg, it->second, planes(i,0), costBuffers[j]);
                    if (boxFilterBeforeOcclusion && matchingCosts != PLANE_SWEEP_ZNCC)
                    {
                        planeSweepBoxFilterCosts(costBuffers[j], boxFilterTempBuffer, matchWindowRadiusX, matchWindowRadiusY);
                        DeviceBuffer<float> temp = boxFilterTempBuffer;
                        boxFilterTempBuffer = costBuffers[j]; // swap buffers
                        costBuffers[j] = temp;
                    }
                    break;
            }
            case PLANE_SWEEP_OCCLUSION_REF_SPLIT:
            {
                float H[9];
                planeHomography(planes(i,0), refImg.cam, it->second.cam, H);

                switch (matchingCosts)
                {
                case PLANE_SWEEP_SAD:
                    if (it->first < refImgId)
                    {
                        planeSweepWarpADAccum(it->second.devImg, colorMatchingEnabled, H,
                                              refImg.devImg, (float) accumScale0, costAccumBeforeBuffer);
                    }
                    else
                    {
                        planeSweepWarpADAccum(it->second.devImg, colorMatchingEnabled, H,
                                              refImg.devImg, (float) accumScale1, costAccumAfterBuffer);
                    }
                    break;
                case PLANE_SWEEP_ZNCC:
                    if (it->first < refImgId)
                    {
                        planeSweepWarpZNCCAccum(it->second.devImg, H, refImg.devImg,
                                               refImgBoxFilterBuffer, refImgSqrBoxFilterBuffer,
                                               (float) accumScale0, costAccumBeforeBuffer, matchWindowRadiusX, matchWindowRadiusY);
                    }
                    else
                    {
                        planeSweepWarpZNCCAccum(it->second.devImg, H, refImg.devImg,
                                               refImgBoxFilterBuffer, refImgSqrBoxFilterBuffer,
                                               (float) accumScale1, costAccumAfterBuffer, matchWindowRadiusX, matchWindowRadiusY);
                    }
                    break;
                }

                break;
            }
            }
            j++;
        }

        if (occlusionMode == PLANE_SWEEP_OCCLUSION_BEST_K)
        {
            // accumulate the best K
            DeviceBuffer<float> best = bestKBuffer0;
            DeviceBuffer<float> bestMin = bestKBuffer1;

            const float bestMax = 1e6;

            costAccumBuffer.clear(0);
            best.clear(bestMax);
            bestMin.clear(0);

            for (int k = 0; k < this->occlusionBestK; k++)
            {
                for (unsigned int j = 0; j < costBuffers.size(); j++)
                {
                    updateBestK(costBuffers[j], best, bestMin);
                }

                // accumulate best
                planeSweepAccumCostBestK(costAccumBuffer, best, bestMin, bestMax, (float) accumScale0);

                // swap buffers
                DeviceBuffer<float> temp = bestMin;
                bestMin = best;
                best = temp;
                best.clear(bestMax);
            }
        }
        else if (occlusionMode == PLANE_SWEEP_OCCLUSION_REF_SPLIT)
        {
            if (boxFilterBeforeOcclusion && matchingCosts != PLANE_SWEEP_ZNCC)
            {
                planeSweepBoxFilterCosts(costAccumBeforeBuffer, boxFilterTempBuffer, matchWindowRadiusX, matchWindowRadiusY);
                DeviceBuffer<float> temp = boxFilterTempBuffer;
                boxFilterTempBuffer = costAccumBeforeBuffer;
                costAccumBeforeBuffer = temp;
                planeSweepBoxFilterCosts(costAccumAfterBuffer, boxFilterTempBuffer, matchWindowRadiusX, matchWindowRadiusY);
                temp = boxFilterTempBuffer;
                boxFilterTempBuffer = costAccumAfterBuffer;
                costAccumAfterBuffer = temp;
            }

            // take the minimum of the two buffers
            planeSweepMinFloat(costAccumBeforeBuffer, costAccumAfterBuffer, costAccumBuffer);
        }

        // box filter
        if ((occlusionMode == PLANE_SWEEP_OCCLUSION_NONE || !boxFilterBeforeOcclusion) && matchingCosts != PLANE_SWEEP_ZNCC)
        {
            planeSweepBoxFilterCosts(costAccumBuffer, boxFilterTempBuffer, matchWindowRadiusX, matchWindowRadiusY);
            DeviceBuffer<float> temp = boxFilterTempBuffer;
            boxFilterTempBuffer = costAccumBuffer; // swap buffers
            costAccumBuffer = temp;
        }


        if (outputCostVolumeEnabled)
        {
            costAccumBuffer.download((float*)&costVolume(0,0,i), costVolume.getWidth()*sizeof(float));
        }

        if (subPixelEnabled && i >= 2)
        {
            if (outputBestDepthEnabled || outputUniquenessRatioEnabled)
            {
                if (outputUniquenessRatioEnabled)
                {
                    planeSweepUpdateBestAndSecondBestPlaneSubPixel(costAccumBuffer, subPixelCostAccumBufferPrev1, subPixelCostAccumBufferPrev2, refImg.devImg.getWidth(), refImg.devImg.getHeight(), i-1,
                                                                   bestPlaneCostBuffer, secondBestPlaneCostBuffer, bestPlaneBuffer, subPixelPlaneOffsetsBuffer);
                }
                else
                {
                    planeSweepUpdateBestPlaneSubPixel(costAccumBuffer, subPixelCostAccumBufferPrev1, subPixelCostAccumBufferPrev2, refImg.devImg.getWidth(), refImg.devImg.getHeight(), i-1,
                                                      bestPlaneCostBuffer, bestPlaneBuffer, subPixelPlaneOffsetsBuffer);
                }
            }
        }

        if (!subPixelEnabled || (i == 0 || i == numPlanes-1))
        {
            if (outputBestDepthEnabled || outputUniquenessRatioEnabled)
            {
                if (outputUniquenessRatioEnabled)
                {
                    planeSweepUpdateBestAndSecondBestPlane(costAccumBuffer, refImg.devImg.getWidth(), refImg.devImg.getHeight(), i,
                                                           bestPlaneCostBuffer, secondBestPlaneCostBuffer, bestPlaneBuffer);

                }
                else
                {
                    planeSweepUpdateBestPlane(costAccumBuffer, refImg.devImg.getWidth(), refImg.devImg.getHeight(), i,
                                              bestPlaneCostBuffer, bestPlaneBuffer);
                }
            }
        }

        if (subPixelEnabled)
        {
            // rotate buffers
            PSL_CUDA::DeviceBuffer<float> temp = subPixelCostAccumBufferPrev2;
            subPixelCostAccumBufferPrev2 = subPixelCostAccumBufferPrev1;
            subPixelCostAccumBufferPrev1 = costAccumBuffer;
            costAccumBuffer = temp;
        }
    }

    if (outputBestDepthEnabled)
    {
        // store planes in a vector to upload them to gpu
        vector<float> planesVec;
        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < numPlanes; i++)
            {
                planesVec.push_back((float) planes(i,0)(j));
            }
        }

        // get inverse ref camera matrix
        Matrix<double, 3, 3> KrefInv = refImg.cam.getK().inverse();
        vector<float> KrefInvVec;
        for (int i=0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                KrefInvVec.push_back((float) KrefInv(i,j));
            }
        }

        bestDepth = DepthMap<float, double>(refImg.devImg.getWidth(), refImg.devImg.getHeight(), refImg.cam);

        if (subPixelEnabled)
        {
            switch(subPixelInterpMode)
            {
            case PLANE_SWEEP_SUB_PIXEL_INTERP_INVERSE:
                planeSweepComputeBestDepthsSubPixelInverse(bestPlaneBuffer, subPixelPlaneOffsetsBuffer, numPlanes, planesVec,
                                                           bestDepth.getDataPtr(), bestDepth.getWidth()*sizeof(float), KrefInvVec);
                break;
            case PLANE_SWEEP_SUB_PIXEL_INTERP_DIRECT:
                planeSweepComputeBestDepthsSubPixelDirect(bestPlaneBuffer, subPixelPlaneOffsetsBuffer, numPlanes, planesVec,
                                                          bestDepth.getDataPtr(), bestDepth.getWidth()*sizeof(float), KrefInvVec);
                break;
            }
        }
        else
        {
            planeSweepComputeBestDepths(bestPlaneBuffer, numPlanes, planesVec,
                                        bestDepth.getDataPtr(), bestDepth.getWidth()*sizeof(float), KrefInvVec);
        }
    }

    if (outputBestCostsEnabled)
    {
        bestCosts = Grid<float>(bestPlaneCostBuffer.getWidth(), bestPlaneCostBuffer.getHeight());
        bestPlaneCostBuffer.download(bestCosts.getDataPtr(), bestCosts.getWidth()*sizeof(float));
    }

    if (outputUniquenessRatioEnabled)
    {
        computeUniquenessRatio(bestPlaneCostBuffer, secondBestPlaneCostBuffer, secondBestPlaneCostBuffer);
        uniqunessRatios = Grid<float>(bestPlaneCostBuffer.getWidth(), bestPlaneCostBuffer.getHeight());
        secondBestPlaneCostBuffer.download(uniqunessRatios.getDataPtr(), uniqunessRatios.getWidth()*sizeof(float));
    }
}

void CudaPlaneSweep::process(int refImgId)
{
    // check if values are set
    if (!(nearZ > 0 && farZ > 0 && numPlanes > 0))
    {
        PSL_THROW_EXCEPTION("Parameters not set properly")
    }

    CudaPlaneSweepImage refImg;

    if (images.count(refImgId) == 1)
    {
        refImg = images[refImgId] ;
    }
    else
    {
        stringstream strstream;
        strstream << "Image with ID " << refImgId << " does not exist.";
        PSL_THROW_EXCEPTION(strstream.str().c_str());
    }

    planes = Grid<Vector4d>(numPlanes,1);

    switch(planeGenerationMode)
    {
    case PLANE_SWEEP_PLANEMODE_UNIFORM_DEPTH:
    {
        double step = (farZ - nearZ)/(numPlanes-1);

        for (int i = 0; i < numPlanes; i++)
        {
            planes(i,0).setZero();
            planes(i,0)(2) = -1;
            planes(i,0)(3) = nearZ + i*step;
        }
        break;
    }
    case PLANE_SWEEP_PLANEMODE_UNIFORM_DISPARITY:
    {
        double maxB = largestBaseline(refImg);

        double avgF = (refImg.cam.getK()(0,0)+refImg.cam.getK()(1,1))/2.0;

        double minD = maxB*avgF/farZ;
        double maxD = maxB*avgF/nearZ;

        double dStep = (maxD - minD)/(numPlanes-1);

        for (int i = 0; i < numPlanes; i++)
        {
            planes(i,0).setZero();
            planes(i,0)(2) = -1;
            planes(i,0)(3) = maxB*avgF/(maxD - i*dStep);
        }
        break;
    }
    }

    process(refImgId, planes);
}

void CudaPlaneSweep::planeHomography(const Matrix<double, 4, 1>& plane, const CameraMatrix<double> &refCam, const CameraMatrix<double> &otherCam, float* H)
{
    const Matrix<double, 3, 3> kRef = refCam.getK();
    const Matrix<double, 3, 3> kOther = otherCam.getK();
    const Matrix<double, 3, 3> rRef = refCam.getR();
    const Matrix<double, 3, 3> rOther = otherCam.getR();
    const Matrix<double, 3, 1> tRef = refCam.getT();
    const Matrix<double, 3, 1> tOther = otherCam.getT();
    const Matrix<double, 3, 1> n = plane.head(3);
    const Matrix<double, 3, 3> rRel = rOther*rRef.transpose();
    const Matrix<double, 3, 1> tRel = rRel*tRef - tOther;
    Matrix<double, 3, 3> Hmat = kOther*(rRel + ((double) 1)/plane(3) * tRel*n.transpose())*kRef.inverse();

    H[0] = (float) Hmat(0,0); H[1] = (float) Hmat(0,1); H[2] = (float) Hmat(0,2);
    H[3] = (float) Hmat(1,0); H[4] = (float) Hmat(1,1); H[5] = (float) Hmat(1,2);
    H[6] = (float) Hmat(2,0); H[7] = (float) Hmat(2,1); H[8] = (float) Hmat(2,2);
}

void CudaPlaneSweep::enableOutputBestDepth(bool enabled)
{
    outputBestDepthEnabled = enabled;
}

void CudaPlaneSweep::enableOutputBestCosts(bool enabled)
{
    outputBestCostsEnabled = enabled;
}

DepthMap<float, double> CudaPlaneSweep::getBestDepth()
{
    return bestDepth;
}

CameraMatrix<double> CudaPlaneSweep::getRefImgCam()
{
    return refImgCam;
}

Grid<float> CudaPlaneSweep::getBestCosts()
{
    return bestCosts;
}

// if color matching was previously disabled all images will be deleted
void CudaPlaneSweep::enableColorMatching(bool enabled)
{
    if (enabled && matchingCosts == PLANE_SWEEP_ZNCC)
    {
        PSL_THROW_EXCEPTION( "NCC not supported with color matching" )
    }

    if (colorMatchingEnabled != enabled)
    {
        // delete all images

        vector<int> idsToDelete;

        for (map<int, CudaPlaneSweepImage>::iterator it = images.begin(); it != images.end(); it++)
        {
            idsToDelete.push_back(it->first);
        }

        for (size_t i=0; i < idsToDelete.size(); i++)
        {
            deleteImage(idsToDelete[i]);
        }

        colorMatchingEnabled = enabled;
    }
}

void CudaPlaneSweep::matchImage(const CudaPlaneSweepImage &refImg, const CudaPlaneSweepImage &otherImg, const Matrix<double, 4, 1> &plane, DeviceBuffer<float>& costBuffer)
{
    float H[9];
    planeHomography(plane, refImg.cam, otherImg.cam, H);

    switch(matchingCosts)
    {
    case PLANE_SWEEP_SAD:
        planeSweepWarpAD(otherImg.devImg, colorMatchingEnabled, H, refImg.devImg, costBuffer);
        break;
    case PLANE_SWEEP_ZNCC:
        planeSweepWarpZNCC(otherImg.devImg, H, refImg.devImg,
                          refImgBoxFilterBuffer, refImgSqrBoxFilterBuffer,
                          costBuffer, matchWindowRadiusX, matchWindowRadiusY);
        break;
    }
}

void CudaPlaneSweep::setPlaneGenerationMode(PlaneSweepPlaneGenerationMode planeGenerationMode)
{
    this->planeGenerationMode = planeGenerationMode;
}

void CudaPlaneSweep::enableBoxFilterBeforeOcclusion(bool enabled)
{
    this->boxFilterBeforeOcclusion = enabled;
}


void CudaPlaneSweep::setMatchingCosts(PlaneSweepMatchingCosts matchingCosts)
{
    if (colorMatchingEnabled && matchingCosts == PLANE_SWEEP_ZNCC)
    {
        PSL_THROW_EXCEPTION( "ZNCC not supported with color matching" )
    }

    this->matchingCosts = matchingCosts;
}

void CudaPlaneSweep::setScale(double scale)
{
    this->scale = scale;
}

void CudaPlaneSweep::enableOuputUniquenessRatio(bool enabled)
{
    this->outputUniquenessRatioEnabled = enabled;
}

Grid<float> CudaPlaneSweep::getUniquenessRatios()
{
    return uniqunessRatios;
}

void CudaPlaneSweep::enableOutputCostVolume(bool enabled)
{
    this->outputCostVolumeEnabled = enabled;
}

Grid<float> CudaPlaneSweep::getCostVolume()
{
    return costVolume;
}

Grid<Vector4d> CudaPlaneSweep::getPlanes()
{
    return planes;
}

void CudaPlaneSweep::setSubPixelInterpolationMode(PlaneSweepSubPixelInterpMode interpMode)
{
    subPixelInterpMode = interpMode;
}

void CudaPlaneSweep::enableSubPixel(bool enabled)
{
    this->subPixelEnabled = enabled;
}


void CudaPlaneSweep::updateCameraMatrix(int id, CameraMatrix<double>& cam)
{

    if (images.count(id) != 1)
    {
        stringstream strstream;
        strstream << "Image with ID " << id << " does not exist.";
        PSL_THROW_EXCEPTION(strstream.str().c_str());
    }

    images[id].cam = cam;
}

cv::Mat CudaPlaneSweep::downloadImage(int id)
{
    if (images.count(id) != 1)
    {
        std::stringstream errorMsg;
        errorMsg << "Cannot download image with ID " << id << ". ID invalid.";
        PSL_THROW_EXCEPTION(errorMsg.str().c_str());
    }
    cv::Mat image;
    images[id].devImg.download(image);
    return image;
}

void CudaPlaneSweep::deallocateBuffers()
{
    if (costAccumBuffer.getAddr() != 0)
        costAccumBuffer.deallocate();

    if (subPixelCostAccumBufferPrev1.getAddr() != 0)
        subPixelCostAccumBufferPrev1.deallocate();

    if (subPixelCostAccumBufferPrev2.getAddr() != 0)
        subPixelCostAccumBufferPrev2.deallocate();

    if (subPixelPlaneOffsetsBuffer.getAddr() != 0)
        subPixelPlaneOffsetsBuffer.deallocate();

    if (boxFilterTempBuffer.getAddr() != 0)
        boxFilterTempBuffer.deallocate();

    if (bestPlaneBuffer.getAddr() != 0)
        bestPlaneBuffer.deallocate();

    if (bestPlaneCostBuffer.getAddr() != 0)
        bestPlaneCostBuffer.deallocate();

    if (secondBestPlaneCostBuffer.getAddr() != 0)
        secondBestPlaneCostBuffer.deallocate();

    for (unsigned int i = 0; i < costBuffers.size(); i++)
    {
        if (costBuffers[i].getAddr() != 0)
            costBuffers[i].deallocate();
    }

    if (bestKBuffer0.getAddr() != 0)
        bestKBuffer0.deallocate();

    if (bestKBuffer1.getAddr() != 0)
        bestKBuffer1.deallocate();


    if (costAccumBeforeBuffer.getAddr() != 0)
        costAccumBeforeBuffer.deallocate();

    if (costAccumAfterBuffer.getAddr() != 0)
        costAccumAfterBuffer.deallocate();

    if (refImgBoxFilterBuffer.getAddr() != 0)
        refImgBoxFilterBuffer.deallocate();

    if (otherImgBoxFilterBuffer.getAddr() != 0)
        otherImgBoxFilterBuffer.deallocate();

    if (refImgSqrBoxFilterBuffer.getAddr() != 0)
        refImgSqrBoxFilterBuffer.deallocate();

    if (otherImgSqrBoxFilterBuffer.getAddr() != 0)
        otherImgSqrBoxFilterBuffer.deallocate();

    if (imgProdBoxFilterBuffer.getAddr() != 0)
        imgProdBoxFilterBuffer.deallocate();

    if (boxFilterSqrTempBuffer.getAddr() != 0)
        boxFilterSqrTempBuffer.deallocate();

    if (boxFilterProdTempBuffer.getAddr() != 0)
        boxFilterProdTempBuffer.deallocate();
}
