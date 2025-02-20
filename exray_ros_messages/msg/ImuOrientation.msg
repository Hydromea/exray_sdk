std_msgs/Header header                                       # Standard ROS header for timestamp and frame_id.
geometry_msgs/Quaternion quaternion                          # Quaternion representing orientation reported by the IMU.
geometry_msgs/Quaternion compass_quaternion                  # Quaternion representing orientation with compass component.
exray_ros_messages/Accuracy orientation_accuracy             # Accuracy status of the orientation estimate.
exray_ros_messages/Accuracy compass_orientation_accuracy     # Accuracy status of the compass orientation estimate.