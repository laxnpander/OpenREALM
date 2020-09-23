# Changelog

##### 09/2020: OpenVSLAM as default SLAM
As of today OpenREALM is relying on OpenVSLAM as default visual SLAM framework. The overall performance differences 
between OpenVSLAM and ORB SLAM2 are neglectable, however OpenVSLAM is still actively maintained. You can pull the
current main build without any other modifications. Open new issues in case you encounter any problems, we will do our
best to fix it as fast as possible.

##### 05/2020: Migration to CMAKE only
From now on this repository will contain OpenREALM as CMake only project. We decided to take this step
due to several reasons, but mainly to make it more independet from the ROS1 build system. This will allow it to move to
other platforms (e.g. Windows) or transportation frameworks (e.g. ROS2) in the future with less trouble.

In case you still want to use ROS1 with OpenREALM you are free to do it. Refer to the instructions on the following repository:

https://github.com/laxnpander/OpenREALM_ROS1_Bridge