# The type mask specifies which input should be used for angular targets:
# If a bit is set, the corresponding target is treated as ATTITUDE. Otherwise as RATE (angular velocity).
#   - bit 1 (0x01): Roll — Set to use roll rate target, unset to use roll attitude target.
#   - bit 2 (0x02): Pitch — Set to use pitch rate target, unset to use pitch attitude target.
#   - bit 3 (0x04): Yaw — Set to use yaw rate target, unset to use yaw attitude target.
#   - bit 4-7: Reserved.
#   - bit 8 (0x80): If set, then rate targets is used without internal control, using directly inverse kinematics 
#                   to convert rate targets to motor commands.

std_msgs/Header header                     # Standard ROS header for timestamp and frame_id.
uint8 type_mask                            # Type mask.
geometry_msgs/Quaternion attitude_delta    # Delta attitude quaternion to reconstruct target from current estimate.
geometry_msgs/Vector3 angular_rate         # Angular rates for roll, pitch and yaw given in rad/s.

# Constant values for the type mask
uint8 TARGET_ATTITUDE_MASK = 0                           # (00000000): Mask for attitude control.
uint8 TARGET_RATE_MASK = 7                               # (00000111): Mask for rate control.
uint8 TARGET_MANUAL_CONTROL_MASK = 128                   # (10000000): Mask for mapping rate targets directly to motor commands.