std_msgs/Header header          # Standard ROS header containing timestamp and frame ID.
int64 time_of_report            # Timestamp of the report (Unix timestamp).
geometry_msgs/Vector3 position  # 3D vectors containing position values in the FLU locale coordinate frame (m).
float64 std                     # Standard deviation (figure of merit) for position (m).
float64 roll                    # Roll angle (degrees).
float64 pitch                   # Pitch angle (degrees).
float64 yaw                     # Yaw angle (degrees).
uint8 status                    # Status code: 0 for normal operation, 1 indicating an issue.
bool ntp_synced                 # True if time is synchronized with NTP, False otherwise.