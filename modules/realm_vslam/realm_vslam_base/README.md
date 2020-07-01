# REALM Visual SLAM package

This package contains a wrapper for visual slam implementations for aerial imagery usage.
It is also able to calculate scale and georeference image data based on GNSS data only.
Most important info is to provide the appropriate input data, here defined as "GeoimageCompressed".

Current SLAM implementations supported:
- ORB SLAM 2

## Install

On first compile:
- chmod +x tools/bin_vocabulary
- Execute tools/bin_vocabulary

**Note: Make sure you sourced the package directory (devel/setup.bash)**

## General structure

Step by step idea is as follows:
1) First grab image and GNSS data and pack together as "frame"
2) Try to initialize monocular SLAM
3) As soon as visual SLAM is initialized, try to calculate transformation between local SLAM coordinate system and global geo coordinate system (defined as ENU) as follows:
    - Compute average scale based on quotient of current GNSS position and visual position to previous measurements
    - Compute average reference plane normal
    - More than 3 measurements in buffer? Yes -> go on / No -> return
    - Has scale converged below limit? Has reference plane normal converged below limit?
    - If all yes, then initialize reference plane coordinate system
    - Compute 2D-similiarity transformation of visual pose in reference plane to GNSS (utm32) east/north (no altitude)
    - Compute total transformation from local coordinate system to ENU
4) Apply georeference to pose and mappoints of frame
5) Publish frame

## Usage

