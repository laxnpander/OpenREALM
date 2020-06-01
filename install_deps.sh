# Update the Apt Cache
sudo apt update

# Catkin tools for build process
sudo apt install -y -q python-catkin-tools git wget g++ gcc

# Additional ROS package dependencies
sudo apt install -y -q ros-$ROS_DISTRO-geographic-msgs
sudo apt install -y -q ros-$ROS_DISTRO-geodesy
sudo apt install -y -q ros-$ROS_DISTRO-cv-bridge
sudo apt install -y -q ros-$ROS_DISTRO-rviz
sudo apt install -y -q ros-$ROS_DISTRO-pcl-ros

# Eigen3 for several linear algebra problems
sudo apt install -y -q libeigen3-dev

# Gdal library for conversions between UTM and WGS84
sudo apt install -y -q gdal-bin

# Cgal library for delauney 2.5D triangulation and mesh creation
sudo apt install -y -q libcgal-dev
sudo apt install -y -q libcgal-qt5-dev

# PCL for writing point clouds and mesh data
sudo apt install -y -q libpcl-dev

# Exiv2 for Exif tagging.
sudo apt install -y -q exiv2 libexiv2-dev apt-utils

# Used by Pangolin/OpenGL
sudo apt install -y -q libglew-dev libxkbcommon-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev
sudo apt install -y -q libxi-dev libxmu-dev libxmu-headers x11proto-input-dev

# Pangolin

cd ~ && mkdir Pangolin && cd Pangolin
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build && cmake ..
make -j $(nproc --all) && sudo make install
