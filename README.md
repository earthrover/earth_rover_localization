# earth_rover_localisation

Earth Rover localisation packages

This package implements the localisation for the earth rover robot. 

## Dependencies
To succesfully compile this package you need:

Real time implementation
[robot_localization](http://docs.ros.org/kinetic/api/robot_localization/html/index.html): Robot Localization is a collection of state estimation nodes, each of which is an implementation of a nonlinear state estimator for robots moving in 3D space. It contains two state estimation nodes, ekf_localization_node and ukf_localization_node. In addition, robot_localization provides navsat_transform_node, which aids in the integration of GPS data.
[Piksy Driver](https://support.swiftnav.com/customer/en/portal/articles/2924342-using-ros-with-swift-navigation-gnss-devices): Driver from ETH Zurich compatible with the Swift Navigation devices 
[xsense Driver](http://wiki.ros.org/xsens_driver): Driver for the third and fourth generation of Xsens IMU devices

PC host to monitorize results on GUI
[Mapviz](https://github.com/swri-robotics/mapviz): Visualization tool with a plug-in system similar to RVIZ focused on visualizing 2D data.

## Installation

Create a workspace, clone the repository and compile it:
	$ mkdir -p ~/er_ws/src  &&	cd ~/er_ws/src 	
	$ git clone https://github.com/earthrover/earth_rover_localisation.git
	$ cd .. & catkin_make
	$ source devel/setup.bash

## Use this package

The package includes .bag example files of recorded tracks to run robot localization and tune the EKF params if necessary
The following launch file reproduces a bag file and applies the robot localization to produce the pose estimation of the rover in a map

	$ roslaunch er_localisation er_localisation.launch

### Input

The main topics published by the bag player to use on robot localization are:
- /mti/sensor/imu: A [sensor_msgs/Imu.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) with the imu data
- /heading: A [sensor_msgs/Imu.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) which reports the vehicle heading reported by the baseline between reference and attitude receiver
- /piksi_receiver/navsatfix_best_fix: A [sensor_msgs/NavSatFix.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html) that contains WGS 84 coordinates with best available fix at the moment (either RTK or SBAS)

### Nodes
Three nodes are used on the architecture
- heading_listener: In charge of traducing the vehicle heading from the baseline into ROS-compliant message to use in robot localization node
- navsat_transform: Takes as input the GPS data and produces an odometry message in coordinates that are consistent with the robot’s world frame. More info on http://docs.ros.org/jade/api/robot_localization/html/navsat_transform_node.html.
- ekf_localization: Robot localization node.. 

### Outputs

[nav_msgs/Odometry.msg](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html) 
