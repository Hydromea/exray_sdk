std_msgs/Header header
float32 depth 0.0 # in meters below surface
float32 temperature 0.0 # in celsius
int16 heading 0 # in degrees (0 is north)
string mission ""
string section ""
string status ""
int16 rov_sysid 0
int8 control # 0 is passive, 1 is active manual, 2 can be active automated
int8 camera_tilt 0
int8 camera_zoom 0
float32 roll 
float32 pitch
float32 yaw
string vertical_mode