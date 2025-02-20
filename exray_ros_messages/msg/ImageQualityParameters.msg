# This message is used to configure camera settings dynamically.
# Each value defaults to -1, which means the corresponding setting should remain unchanged.
# This allows selective adjustment of specific parameters without requiring multiple services.

std_msgs/Header header                # Standard ROS header for timestamp and frame_id
int8 brightness -1                    # Adjust brightness (0-100). Use -1 to retain the current value.
int8 contrast -1                      # Adjust contrast (0-100). Use -1 to retain the current value
int8 sharpness -1                     # Adjust sharpness (0-100). Use -1 to retain the current value.
int8 saturation -1                    # Adjust color saturation (0-100). Use -1 to retain the current value.
int16 iso -1                          # Set ISO sensitivity (100-800). Use -1 to retain the current value.
bool active_brightness_control false  # Enable automatic brightness control if true.
bool active_contrast_control false    # Enable automatic contrast control if true.
int8 auto_white_balance_mode -1       # Auto white balance mode (see constants below)
int8 metering_mode -1                 # Metering mode for exposure (see constants below).
int8 exposure_mode -1                 # Exposure mode (see constants below).
int8 image_effect -1                  # Image effect to apply (see constants below).

# Constants for Auto White Balance Modes
int8 CAMERA_AWB_IGNORE       = -1
int8 CAMERA_AWB_OFF          = 0
int8 CAMERA_AWB_AUTO         = 1
int8 CAMERA_AWB_SUNLIGHT     = 2
int8 CAMERA_AWB_CLOUDY       = 3
int8 CAMERA_AWB_SHADE        = 4
int8 CAMERA_AWB_TUNGSTEN     = 5
int8 CAMERA_AWB_FLUORESCENT  = 6
int8 CAMERA_AWB_INCANDESCENT = 7
int8 CAMERA_AWB_FLASH        = 8
int8 CAMERA_AWB_HORIZON      = 9

# Constants for Metering Modes
int8 CAMERA_METERING_IGNORE  = -1
int8 CAMERA_METERING_AVERAGE = 0
int8 CAMERA_METERING_SPOT    = 1
int8 CAMERA_METERING_BACKLIT = 2
int8 CAMERA_METERING_MATRIX  = 3

# Constants for Exposure Modes 
int8 CAMERA_EXPOSURE_IGNORE       = -1
int8 CAMERA_EXPOSURE_OFF          = 0
int8 CAMERA_EXPOSURE_AUTO         = 1
int8 CAMERA_EXPOSURE_NIGHT        = 2
int8 CAMERA_EXPOSURE_NIGHTPREVIEW = 3
int8 CAMERA_EXPOSURE_BACKLIGHT    = 4
int8 CAMERA_EXPOSURE_SPOTLIGHT    = 5
int8 CAMERA_EXPOSURE_SPORTS       = 6
int8 CAMERA_EXPOSURE_SNOW         = 7
int8 CAMERA_EXPOSURE_BEACH        = 8
int8 CAMERA_EXPOSURE_VERYLONG     = 9
int8 CAMERA_EXPOSURE_FIXEDFPS     = 10
int8 CAMERA_EXPOSURE_ANTISHAKE    = 11
int8 CAMERA_EXPOSURE_FIREWORKS    = 12

# Constants for image effect
int8 CAMERA_IMAGE_EFFECT_IGNORE       = -1
int8 CAMERA_IMAGE_EFFECT_NONE         = 0
int8 CAMERA_IMAGE_EFFECT_NEGATIVE     = 1
int8 CAMERA_IMAGE_EFFECT_SOLARIZE     = 2
int8 CAMERA_IMAGE_EFFECT_SKETCH       = 3
int8 CAMERA_IMAGE_EFFECT_DENOISE      = 4
int8 CAMERA_IMAGE_EFFECT_EMBOSS       = 5
int8 CAMERA_IMAGE_EFFECT_OILPAINT     = 6
int8 CAMERA_IMAGE_EFFECT_HATCH        = 7
int8 CAMERA_IMAGE_EFFECT_GPEN         = 8
int8 CAMERA_IMAGE_EFFECT_PASTEL       = 9
int8 CAMERA_IMAGE_EFFECT_WATERCOLOR   = 10
int8 CAMERA_IMAGE_EFFECT_FILM         = 11
int8 CAMERA_IMAGE_EFFECT_BLUR         = 12
int8 CAMERA_IMAGE_EFFECT_SATURATION   = 13
int8 CAMERA_IMAGE_EFFECT_COLORSWAP    = 14
int8 CAMERA_IMAGE_EFFECT_WASHEDOUT    = 15
int8 CAMERA_IMAGE_EFFECT_POSTERISE    = 16
int8 CAMERA_IMAGE_EFFECT_COLORPOINT   = 17
int8 CAMERA_IMAGE_EFFECT_COLORBALANCE = 18
int8 CAMERA_IMAGE_EFFECT_CARTOON      = 19

