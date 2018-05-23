# A message to handle georeferenced images projected to a ground plane

# Projected image at ground level in ENU coordinate frame
sensor_msgs/CompressedImage imagedata

# Global position of the image to be displayed. Upper left corner
sensor_msgs/NavSatFix gpsdata

# Scale of warped image with [m / pixels]
float64 scale

