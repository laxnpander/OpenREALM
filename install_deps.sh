# Update the Apt Cache
sudo apt update

# Catkin tools for build process
sudo apt install -y -q python-catkin-tools

# Additional ROS package dependencies
sudo apt install -y -q ros-melodic-geographic-msgs
sudo apt install -y -q ros-melodic-geodesy

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

# Pangolin
echo "Step1"
echo $(pwd)
cd ~
echo "Step2"
echo $(pwd)
mkdir Pangolin
echo "Step3"
echo $(pwd)
cd Pangolin
echo "Step4"
echo $(pwd)
git clone https://github.com/stevenlovegrove/Pangolin.git
echo "Step5"
echo $(pwd)
cd Pangolin
echo "Step6"
echo $(pwd)
mkdir build
echo "Step7"
echo $(pwd)
cd build
echo "Step8"
echo $(pwd)
cmake ..
echo "Step9"
echo $(pwd)
make -j
echo "Step10"
echo $(pwd)
sudo make install
echo "Step11"
echo $(pwd)