std_msgs/Header header       # Standard ROS header containing timestamp and frame ID.
float32 pressure             # Measured raw pressure value in millibars (mbar).
float32 temperature          # Measured raw temperature value in degrees Celsius (°C).
float32 depth                # Calculated depth in meters (m). Negative values indicate the device is above water.
float32 surface_pressure     # Calibrated surface pressure reference in millibars (mbar), used to determine submersion status.
float32 vertical_velocity    # Vertical velocity in meters per second (m/s). Positive value indicates descent in water.
bool submerged               # True if the vehicle is submerged, false otherwise.