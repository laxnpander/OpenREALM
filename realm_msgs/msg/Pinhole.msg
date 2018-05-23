#####################################################
# A message containing general pinhole camera model #
#####################################################

# Image dimensions
std_msgs/UInt32 width
std_msgs/UInt32 height

# Principal point
std_msgs/Float64 cx
std_msgs/Float64 cy

# Focal length
std_msgs/Float64 fx
std_msgs/Float64 fy

# Radial distortion parameters
std_msgs/Float64 k1
std_msgs/Float64 k2
std_msgs/Float64 p1
std_msgs/Float64 p2
std_msgs/Float64 k3
