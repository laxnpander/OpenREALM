# Changelog

##### 05/2020: Migration to CMAKE only
From now on this repository will contain OpenREALM as CMake only project. We decided to take this step
due to several reasons, but mainly to make it more independet from the ROS1 build system. This will allow it to move to
other platforms (e.g. Windows) or transportation frameworks (e.g. ROS2) in the future with less trouble.

In case you still want to use ROS1 with OpenREALM you are free to do it. Refer to the instructions on the following repository:

https://github.com/laxnpander/OpenREALM_ROS1_Bridge