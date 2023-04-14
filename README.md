# camera_ros

This is a ROS package of Visual Localization using IR camera and landmarks.
This package is based on works of @zhangqr fusionLocalization.

Use "roslaunch camera_ros camera_controller" to start positioning.
Different resolution supported, but you must change params first. 
It is not likely to use 1080*1920 on smart car. Use 480*640 instead, which will process faster.

Files in /copy_trash dir is unused.