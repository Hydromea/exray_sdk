std_msgs/Header header  # Standard ROS header for timestamp and frame_id.
uint8 control_mode      # Mode of the control.
uint8 rov_mode          # Mode of the rov (1 for flyout, 0 for mothership).
uint16 buttons          # Bitwise status of buttons.
uint16 switches         # Bitwise status of switches.
int8 linear_x           # Linear component x of the joystick input (in FLU format) -- value bewteen -100 and +100.
int8 linear_y           # Linear component y of the joystick input (in FLU format) -- value bewteen -100 and +100.
int8 linear_z           # Linear component z of the joystick input (in FLU format) -- value bewteen -100 and +100.
int8 angular_x          # Angular component x of the joystick input (in FLU format) -- value bewteen -100 and +100.
int8 angular_y          # Angular component y of the joystick input (in FLU format) -- value bewteen -100 and +100.
int8 angular_z          # Angular component z of the joystick input (in FLU format) -- value bewteen -100 and +100.
int8 tilt               # tilt -- value bewteen -100 and +100.
uint8 focus             # focus -- value bewteen 0 and +200.