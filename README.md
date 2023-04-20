# camera_ros

This is a ROS package of Visual Localization using IR camera and landmarks.
This package is based on works of @zhangqr fusionLocalization.

Use "roslaunch camera_ros camera_controller" to start positioning.
Different resolution supported, but you must change params first. 
It is not likely to use 1080*1920 on smart car. Use 480*640 instead, which will process faster.

Files in /copy_trash dir is unused.

In 2023-04, I re-tested this ROS package, and settled params for different resolution.
Detailed params can be found in /screenshots dir. One for 640*480 and one for 1920*1080.
I DON'T recommend you use 1080p on smartcar. It is SUPER SLOW.
USE 480p for smartcar!!!

You can use different main file in CMake.
use this for ROS system:
add_executable(camera_controller 
src/Camera_ros_node.cpp
......)
or use this for single camera testing:
add_executable(camera_controller 
src/main.cpp
......)