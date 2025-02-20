std_msgs/Header header      # Standard ROS header containing timestamp and frame ID.
float32 angle               # Measurement angle in radians [rad].
uint8 gain                  # Sonar gain level (0-2), used to control signal amplification.
uint16 number_of_samples    # Number of samples in the sonar intensity array.
uint16 transmit_frequency   # Transmit frequency of the sonar [kHz].
uint16 speed_of_sound       # Speed of sound in the medium [m/s].
uint8 range                 # Sonar range value in meters [m].
uint8[] intensities         # Intensity data array [0-255]. Represents echo intensity received by the sonar.
