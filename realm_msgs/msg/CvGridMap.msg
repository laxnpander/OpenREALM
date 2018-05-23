# Header for time and frame
Header header

# CvGridMap uni resolution in x-/y-dir
float64 resolution

# CvGridMap position of roi
geometry_msgs/Point pos

# CvGridMap length of roi in x
float64 length_x

# CvGridMap length of roi in y
float64 length_y

# CvGridMap layer names
string[] layers

# CvGridMap layer data
sensor_msgs/Image[] data
