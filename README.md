
<p align="center">
  <img alt="OpenREALM Logo" src="https://github.com/laxnpander/OpenREALM/blob/master/resources/imgs/logo.png?raw=true">
</p>

This is the repository for Open REALM, a real-time aerial mapping framework. It is currently in alpha state, so don't expect
a polished and bugfree application. 

Feel free to fork and contribute. Let's make mapping fast again! :)

## Changelog

**01/2021: Licensing**

Finally some good news for 2021: Most of the OpenREALM library will now be available under the LGPL license. Yay! This 
allows commercial users to integrate our code base into their products without having the need to publish all 
infrastructure around it. However, changes to the original code base have to be made available to all users. 

Please note: If your are compiling with CUDA and consequently use the thirdparty PSL library, the whole code base becomes GPLv3!

**09/2020: OpenVSLAM as default SLAM**

As of today OpenREALM is relying on OpenVSLAM as default visual SLAM framework. The overall performance differences 
between OpenVSLAM and ORB SLAM2 are neglectable, however OpenVSLAM is still maintained. At this point we use a custom
version of OpenVSLAM though, as we need the tracker state for which there is no getter in the current master build.
Until [this PR](https://github.com/xdspacelab/openvslam/pull/399) is not merged please use 
[this](https://github.com/laxnpander/openvslam) or the provided install script in tools/install_deps.sh.

**05/2020: Migration to CMAKE only**

Please note, that from now on this repository will contain OpenREALM as CMake only project. We decided to take this step
due to several reasons, but mainly to make it more independet from the ROS1 build system. This will allow it to move to
other platforms (e.g. Windows) or transportation frameworks (e.g. ROS2) in the future with less trouble.

In case you still want to use ROS1 with OpenREALM you are free to do it. Refer to the instructions on the following repository:

https://github.com/laxnpander/OpenREALM_ROS1_Bridge

## Demonstration

[![Demonstration Video](https://img.youtube.com/vi/9MvPTHP0r0c/0.jpg)](https://www.youtube.com/watch?v=9MvPTHP0r0c)

## Related Publication

If you use OpenREALM for your research, please cite:

A. Kern, M. Bobbe, Y.Khedar and U.Bestmann. OpenREALM: Real-time Mapping for Unmanned Aerial Vehicles. [Preprint](https://arxiv.org/pdf/2009.10492.pdf)

```
@inproceedings{9213960,
  author    =     {A. {Kern} and M. {Bobbe} and Y. {Khedar} and U. {Bestmann}},
  title     =     {OpenREALM: Real-time Mapping for Unmanned Aerial Vehicles},
  booktitle =     {2020 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
  year      =     {2020},
  volume    =     {},
  number    =     {},
  pages     =     {902-911},
  doi       =     {10.1109/ICUAS48674.2020.9213960},
}
```

A more detailed outline can be found in my [master thesis](https://drive.google.com/file/d/1Xpsdc02y9oKwY50ZrjYIxh_weNuoM-Ww/view?usp=sharing).

## Introduction

The proposed framework stands on the shoulders of giants, namely the open source implementation for
visual SLAM and stereo reconstruction. Please read the references below. 

**Beforehand**

The pipeline is designed for multirotor systems with downlooking 2-axis gimbal stabilized camera and alignment of the
magnetic heading of the UAV with the image's negative y-axis. It should be easily extendable to other cases in the future,
but at the moment I won't recommend to use it on any other.

**What it can do**
- 2D maps based on lat, lon, alt and heading informations only
- 2D maps based on lat, lon, alt and visual SLAM pose estimation
- 2.5D elevation maps based on lat, lon, alt and visual SLAM pose estimation using sparse cloud
- 2.5D elevation maps based on lat, lon, alt and visual SLAM pose estimation using GPU stereo reconstruction
- All of the above from a single or multiple UAVs

**What NOT to expect**
- Transportation of your data from the UAV to the ground based on any other than ROS
- Don't expect magic, mapping with visual informations can only be done in regions with actual features. 
  The final map will only be as accurate as the chosen implementation for pose estimation, densification and
  fusion of both in the mosaic.
- Polished, ready-to-use code for commercial applications. It is research code that has developed over a long 
  time and is therefore sometimes ugly and buggy as hell. But I am working on it :)

## Prerequisites

Currently we only support Linux systems. However, we have a strong determination to provide an architecture independent
solution in the close future.

| OS         | Build Status |
|:----------:|:------------:|
|Ubuntu 16.04| [![Build Status](https://travis-ci.org/laxnpander/OpenREALM.svg?branch=master)](https://travis-ci.org/laxnpander/OpenREALM) |
|Ubuntu 18.04| [![Build Status](https://travis-ci.org/laxnpander/OpenREALM.svg?branch=master)](https://travis-ci.org/laxnpander/OpenREALM) |
|Ubuntu 20.04| [![Build Status](https://travis-ci.org/laxnpander/OpenREALM.svg?branch=master)](https://travis-ci.org/laxnpander/OpenREALM) |

## Optional Dependencies

CUDA is optional but will be mandatory for stereo reconstruction with plane sweep lib

-> Refer to https://developer.nvidia.com/cuda-downloads?target_os=Linux

Please note, that installing CUDA can sometimes be troublesome. If you are facing an error like 
```sh
*fatal error: cuda_runtime.h: No such file or directory*
```
often times adding the CUDA directory to the .bashrc does the trick. If you use CUDA 9.0 for example, you should 
```sh
echo 'export CPATH=/usr/local/cuda-9.0/include:$CPATH' >> ~/.bashrc 
```

## Installation

```
# Get the library
git clone https://github.com/laxnpander/OpenREALM.git

# For 3D reconstruction with Plane Sweep Lib (PSL)
git submodule init
git submodule update

# Install the dependencies
cd OpenREALM/tools
chmod u+x install_deps.sh
./install_deps.sh

# Build and install the main library
cd ..
mkdir build && cd build
cmake ..
make all
sudo make install
```

At this point you have installed the OpenREALM main library. You can use it system-wide in your own packages.
But you are here to get your hands dirty, right? To run OpenREALM on a test dataset some more steps are necessary.

Because the pipeline consists of several, independent processing stages which are not able to communicate with each other,
a transportation framework has to be provided. Right now we advise to use our ROS1 bridge package for that job:

```
# Create a ROS workspace for compilation
mkdir catkin_ws && cd catkin_ws
mkdir src && cd src

# Get the ROS package
git clone https://github.com/laxnpander/OpenREALM_ROS1_Bridge.git

# Make sure you are in catkin_ws, not src. Then build it
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
```

In case something goes wrong creating the ROS workspace, please refer to the bridge repository: 

https://github.com/laxnpander/OpenREALM_ROS1_Bridge

If you successfully installed the main library and built the ROS bridge package, you can continue with the Quick Start.

Have fun!

## Quick Start

**Step 1:**
Download the test dataset:

https://drive.google.com/open?id=1-2h0tasI4wzxZKLBbOz3XbJ7f5xlxlMe

**Step 2:**
Unzip the dataset with a tool of your choice, e.g.
```sh
tar -xvzf open_realm_edm_dataset.tar.gz
```

**Step 3:**
We provided as well a set of configuration files in realm_ros/profiles as the corresponding launch files in 
realm_ros/launch to run the test dataset. The only thing you have to do is modify the path in the launch file:
```sh
node pkg="realm_ros" type="realm_exiv2_grabber" name="realm_exiv2_grabber" output="screen"
    param name="config/id" type="string" value="$(arg camera_id)"/>
    param name="config/input" type="string" value="PUT THE TEST DATASET'S ABSOLUTE PATH HERE"/>
    param name="config/rate" type="double" value="10.0"/>
    param name="config/profile" type="string" value="alexa"/>
/node
```
Note: The exiv2 grabber node reads images and exiv2 tags from the provided folder and publishes them 
for the mapping pipeline.

**Step 4:**
Launch the pipeline you want to use.

- GNSS only mapping:
```sh
roslaunch realm_ros alexa_gnss.launch
```

- 2D mapping with visual SLAM:
```sh
roslaunch realm_ros alexa_noreco.launch
```

- 2.5D mapping with visual SLAM and surface reconstruction:
```sh
roslaunch realm_ros alexa_reco.launch
```
  
## Docker 
OpenREALM can also be used with a docker. The docker is based on Ubuntu 18.04 and all files related to it
are in the ```docker``` folder of this main repository. Testing it is very simple:

**Step 1:** Install Docker

  [Docker Install](https://docs.docker.com/engine/install/)

**Step 2:** Build the Docker image using the script in ```docker``` folder
```bash
  /bin/bash docker_build.sh
```

**Step 3:** Run the Docker image using the script in ```docker``` folder
```bash
  /bin/bash docker_run.sh
```
This script can be run from any folder in the host system. The Working Directory will be
mounted in the docker. The dataset should ideally be kept in this same folder. Then change
the path of the dataset in the launch file as described previously and run the test.

## License

Most of the code base is published under LGPL license. This basically includes all functionalities to create 2D maps.
For 3D dense reconstruction however we rely on the thirdparty framework *Plane Sweep Library* by Christian Häne. It is
published under GPLv3, which means the rest of the code base will automatically transition to GPLv3 as well if you compile
it also.

**TL/DR:**
- Using OpenREALM for 2.5/3D maps with CUDA/PSL? -> **GPLv3**
- Using OpenREALM for 2D maps without CUDA/PSL? -> **LGPL**

## References

**Visual SLAM**

[1] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile and Accurate Monocular 
SLAM System. IEEE Transactions on Robotics, vol. 31, no. 5, pp. 1147-1163, 2015. (2015 IEEE Transactions on 
Robotics Best Paper Award).

**Stereo Reconstruction**

[2] Christian Häne, Lionel Heng, Gim Hee Lee, Alexey Sizov, Marc Pollefeys, Real-Time Direct Dense Matching on
Fisheye Images Using Plane-Sweeping Stereo, Proc Int. Conf. on 3D Vison (3DV) 2014

**Other**

[3] P. Fankhauser and M. Hutter, "A Universal Grid Map Library: Implementation and Use Case for Rough Terrain Navigation",
in Robot Operating System (ROS) – The Complete Reference (Volume 1), A. Koubaa (Ed.), Springer, 2016. 

[4] T. Hinzmann, J. L. Schönberger, M. Pollefeys, and R. Siegwart, "Mapping on the Fly: Real-time 3D Dense Reconstruction, Digital Surface Map and Incremental Orthomosaic Generation for Unmanned Aerial Vehicles"
