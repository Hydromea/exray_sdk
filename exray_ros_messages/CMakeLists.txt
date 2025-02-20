cmake_minimum_required(VERSION 3.5)
project(exray_ros_messages)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(msg_files
   "msg/Accelerometer.msg"
   "msg/Accuracy.msg"
   "msg/AttitudeTarget.msg"
   "msg/CameraSteppersParameters.msg"
   "msg/ControlState.msg"
   "msg/CurrentFocusStamped.msg"
   "msg/CurrentTiltStamped.msg"
   "msg/CurrentZoomStamped.msg"
   "msg/DvlBeam.msg"
   "msg/DvlDeadReckoning.msg"
   "msg/DvlVelocity.msg"
   "msg/ExternalPressure.msg"
   "msg/FocalDistanceCalibrated.msg"
   "msg/FocusCommand.msg"
   "msg/Gyroscope.msg"
   "msg/Heartbeat.msg"
   "msg/ImageQualityParameters.msg"
   "msg/ImuBiasWithCovariance.msg"
   "msg/ImuOrientation.msg"
   "msg/JoystickCommand.msg"
   "msg/Magnetometer.msg"
   "msg/MultiPoseSonarScan.msg"
   "msg/PositionTarget.msg"
   "msg/PressureBiasWithCovariance.msg"
   "msg/SonarEcho.msg"
   "msg/Submap.msg"
   "msg/TakePictureCommand.msg"
   "msg/ToggleRecordingCommand.msg"
   "msg/TransformBetweenScans.msg"
   "msg/UsblPing.msg"
   "msg/VideoBitrate.msg"
   "msg/VideoOverlay.msg"   
)

set(srv_files
   "srv/GetCameraSteppersParameters.srv"
   "srv/SetCameraSteppersParameters.srv"
)

set(action_files
   "action/FocusCalibration.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs

)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
