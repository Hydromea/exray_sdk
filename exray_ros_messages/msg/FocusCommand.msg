std_msgs/Header header             # Standard ROS header containing timestamp and frame ID.
sensor_msgs/RegionOfInterest roi   # Region of interest (ROI) to focus on.
float32 zoom -1                    # Desired zoom value; -1 keeps the current zoom level.
string method "fft"                # Sharpness measurement method; currently, only 'fft' is supported.