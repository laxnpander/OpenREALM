Ground Image Plugin for rviz
============================

- the plugin subscribes to a user definde msg (GeoImage)
  and renders the image in the rviz 3d Area

- the GeoImage msg contain a ROS Image, a sensor_msg/Pose and the resolution of the image in pixel per meter

- the pose of the GeoImage msg is set to the top left corner of the image (TODO, check this again)

- the GeoImage pose is in a global cartesian coordinate system, 
  a proper conversion from latitude, longitude to the  global rviz cartesin coordiante system has to be done in a step before



