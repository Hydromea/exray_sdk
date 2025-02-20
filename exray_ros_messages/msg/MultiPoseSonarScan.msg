# Message represents a sonar scan generated with measurements from different sonar's poses (sensor is moving)
# Poses of sonar are represented in scan frame

std_msgs/Header header              # Header info
float32 range                       # Sonar range
geometry_msgs/Point[] points        # If no obstacle detected, set inf in point coordinates
geometry_msgs/Pose[] sensor_poses   # Array of sensor poses (for each measurement)
float32[] angles                      # Angles of scanning sonar head during each measurement