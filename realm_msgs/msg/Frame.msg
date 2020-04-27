#######################################################
# A message containing basic REALM frame informations #
#######################################################

##################### Sensor data #####################
# (required) Header containing message meta data
std_msgs/Header header

# (required) Unique name/id of the acquiring camera
std_msgs/String camera_id

# (required) Frame id which is constant over the whole stage processing. Should only be set once
std_msgs/UInt32 stage_id

# (required) Timestamp of the acquisition. Is not the same as ros message timestamp!
std_msgs/UInt64 timestamp

# (required) Imagedata containing the compressed camera data
sensor_msgs/CompressedImage imagedata

# (required) Camera_model containing pinhole parameters of capturing cam
realm_msgs/Pinhole camera_model

# (required) Gnss data containing geodetic informations of the image
sensor_msgs/NavSatFix gpsdata

# (optional) Point cloud of the observed surface saved as cv::Mat row(i)(x,y,z,r,g,b,nx,ny,nz)
sensor_msgs/Image surface_points

# (optional) Observed grid map of the frame projection
realm_msgs/CvGridMap observed_map

# (optional) Heading in Degree, required if mapping on GNSS only
std_msgs/Float32 heading

# (optional) Visual pose of the camera in local coordinate frame, required e.g. for georeference updates
geometry_msgs/Pose visual_pose

# (optional) Transformation from the visual to the geographic world
realm_msgs/Georeference georeference

######################## Flags ########################
# Reset flag: set true, if following stage should be resetted
std_msgs/Bool do_reset

# Accurate pose flag: set true, if pose is suited for dense reconstruction
std_msgs/Bool has_accurate_pose

# Is key frame flag: set true, if frame was marked as keyframe
std_msgs/Bool is_keyframe

# Is georeferenced flag: set true, if frame was successfully georeferenced
std_msgs/Bool is_georeferenced

# Flag to check if surface was computed with 2.5D elevation or not
std_msgs/Bool is_surface_elevated