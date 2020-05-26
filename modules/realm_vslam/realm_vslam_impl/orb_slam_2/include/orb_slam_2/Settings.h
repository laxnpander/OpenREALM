#ifndef ORBSLAM_SETTINGS_H
#define ORBSLAM_SETTINGS_H

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2 {
  
    //- camera settings
    struct CameraSettings {
	    cv::Mat calibration;
	    cv::Mat distortion;
	    float fps;
	    int rgb;
	    int imgWidth;
	    int imgHeight;
	    float bf;
	    float thDepth;
	    float depthMapFactor;
    };
	
    //- tracker settings
    struct TrackerSettings {
	    int nFeatures;
	    float scaleFactor;
	    int nLevels;
	    int iniThFast;
	    int minThFast;
    };
	
    //- viewer settings
    struct ViewerSettings {
	    float keyFrameSize;
	    float keyFrameLineWidth;
	    float graphLineWidth;
	    float pointSize;
	    float cameraSize;
	    float cameraLineWidth;
	    float viewpointX;
	    float viewpointY;
	    float viewpointZ;
	    float viewpointF;
    };

}

#endif