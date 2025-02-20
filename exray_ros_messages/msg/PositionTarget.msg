std_msgs/Header header                                   # Standard ROS Header, includes timestamp and frame ID.

# The type mask specifies which input should be used for angular targets:
# If a bit is set, the corresponding target is treated as ATTITUDE. Otherwise, it is treated as RATE (angular velocity).
#   - bit 1 (0x01): X — Set to use x-velocity target, unset to use x-position target.
#   - bit 2 (0x02): Y — Set to use y-velocity target, unset to use y-position target.
#   - bit 3 (0x04): Z — Set to use z-velocity target, unset to use z-position target.
#   - bit 4-7: Reserved.
#   - bit 8 (0x80): If set, then velocity target is used without internal control, using directly inverse kinematics 
#                   to convert velocity target to motor commands. (Warning: Overules bit 1-3!).
uint8 type_mask                             # Type mask.
geometry_msgs/Vector3 position              # Position in the robot frame (body frame) in meters (x, y, z).
geometry_msgs/Vector3 velocity              # Velocity in the robot frame (body frame) in meters/second (x, y, z).

# Constant values for the type mask
uint8 TARGET_POSITION_MASK = 0              # (00000000): Mask for position control.
uint8 TARGET_VELOCITY_MASK = 7              # (00000111): Mask for velocity control.
uint8 TARGET_MANUAL_CONTROL_MASK = 128      # (10000000): Mask for mapping velocity targets directly to motor commands.