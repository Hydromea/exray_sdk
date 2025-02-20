std_msgs/Header header   # Standard ROS header containing timestamp and frame ID.
bool record              # Indicates whether to record video: True to start or continue, false to stop.
string path ""           # The root path of the mission where the video will be recorded.
string section ""        # The section within the mission to store the video. If empty, the video is stored in the root mission path.