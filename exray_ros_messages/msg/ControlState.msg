std_msgs/Header header                 # Standard ROS header containing timestamp and frame ID.
geometry_msgs/Vector3 position_lf      # 3D vectors containing position values in the FLU local coordinate frame (m).
geometry_msgs/Vector3 velocity_lf      # 3D vectors containing velocity values in the FLU local coordinate frame (m/s).
geometry_msgs/Vector3 acceleration_lf  # 3D vectors containing acceleration values in the FLU local coordinate frame (m/s^2).
geometry_msgs/Quaternion orientation   # Quaternion representing orientation of the ROV.
geometry_msgs/Vector3 rate             # 3D vectors containing angular speed values (rad/s).
