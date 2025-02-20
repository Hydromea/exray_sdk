std_msgs/Header header           # Standard ROS header containing timestamp and frame ID.
int64 time_of_validity           # Timestamp of surface reflection event (Unix timestamp in microseconds).
int64 time_of_transmission       # Timestamp recorded before sending report over TCP (Unix timestamp in microseconds).
geometry_msgs/Vector3 velocity   # Measured velocity as a 3D vector in meters per second (m/s).
float64 figure_of_merit          # Figure of Merit representing the accuracy of the measured velocities.
float64[9] covariance            # Covariance matrix for the velocity measurements, units in (m/s)^2.
DvlBeam[] beams                  # Array of beam data, defined in DvlBeam.msg.
float64 altitude                 # Altitude along the z-axis, representing distance to the reflecting surface in meters (m).
bool velocity_valid              # True if velocity measurement is valid, False otherwise.
uint8 status                     # Status code: 0 for normal operation, 1 indicating an issue.
bool ntp_synced                  # True if time is synchronized with NTP, False otherwise.