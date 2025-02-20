std_msgs/Header header   # Standard ROS header containing timestamp and frame ID.
float32 range            # Measured distance between the remote and local modem [m].
float32 bearing          # Bearing angle between the remote and local modem [rad].
bool bearing_available   # Indicates if bearing is available.
uint8 remote_id          # Unique ID of the remote modem.
uint8 local_id           # Unique ID of the local modem.