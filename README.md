# Visual Control
Package to control MAVs using Visual Odometry Systems and Computer Vision

## 1. Control Based on Visual Odometry
This package assumes you have a Visual Odometry Package running, such as `orb_slam2_ros` or `svo` (those are the already tested ones)

### Architecture
The node slam_control.py 


To use dynamic_reconfigure, `rosrun rqt_gui rqt_gui -s reconfigure` along with the simple_control.py node
