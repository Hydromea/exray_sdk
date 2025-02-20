std_msgs/Header header       # Standard ROS header containing timestamp and frame ID.
float32 zoom -1              # Zoom level; -1 indicates no change from the current value.
float32 focus -1             # Focus level; -1 indicates no change from the current value.
float32 tilt -1              # Tilt angle; -1 indicates no change from the current value.
float32 focal_distance -1    # Focal distance, may be ignored by control algorithms if, for example, focus is also set or if autofocus is activated.