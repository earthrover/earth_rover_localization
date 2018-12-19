# Earth Rover Localisation

## Dependencies
- The package requires the following dependencies

[robot_localization](http://docs.ros.org/kinetic/api/robot_localization/html/index.html): Robot Localization is a collection of state estimation nodes, each of which is an implementation of a nonlinear state estimator for robots moving in 3D space. It contains two state estimation nodes, ekf_localization_node and ukf_localization_node. In addition, robot_localization provides navsat_transform_node, which aids in the integration of GPS data.

[Piksy Driver](https://github.com/ethz-asl/ethz_piksi_ros): Driver from ETH Zurich compatible with the Swift Navigation devices. 

[xsense Driver](https://github.com/xsens/xsens_mti_ros_node): Driver for the third and fourth generation of Xsens IMU devices

- Dependency on host computer to monitor results.

[Mapviz](https://github.com/swri-robotics/mapviz): Visualization tool with a plug-in system similar to RVIZ focused on visualizing 2D data.

## Installation and Configuration

1. Create a workspace, clone the repository, compile it and source the environment for each terminal you work in.

	```
	$ mkdir -p ~/er_ws/src  &&	cd ~/er_ws/src 	
	$ git clone https://github.com/earthrover/earth_rover_localisation.git
	$ cd .. & catkin_make
	$ source devel/setup.bash
	```

The following steps explain the Hardware and ROS drivers configuration to run in the embedded device and monitor on a host PC. **Only follow this section if the required sensors are available to test for a complete base station - rover setup.** Skip to [Robot Localisation](https://github.com/earthrover/earth_rover_localisation/tree/master/er_localisation#earth-rover-localisation-1) to see robot localisation node, visualization tool and play recorded results.

### Piksy Modules

The Hardware configuration uses three [Piksi Multi Evaluation Board](https://support.swiftnav.com/customer/en/portal/articles/2681333-piksi-multi-evaluation-board): A base station and two receivers on the rover (reference and attitude). 

The reference receiver obtains corrections from base station using the [FreeWave Radio Evaluation Kit](https://support.swiftnav.com/customer/en/portal/articles/2952967-freewave-radio-evaluation-kit) and then send corrections to the attitude receiver which enables precise heading output. 

- Swift console on a host computer

2. Follow the instructions on installing and operating the Swift Console on your computer, see the [Installation Instructions](https://support.swiftnav.com/customer/en/portal/articles/2756825-installing-swift-console) and [User's Guide](https://support.swiftnav.com/customer/en/portal/articles/2838278-swift-console-user-s-guide).

- The following steps to configure on the Piksy modules.

3. Complete the instructions to configure the base station and rover receiver to use the [GNSS RTK Position with Stationary Base solution](https://support.swiftnav.com/customer/en/portal/articles/2771177).

4. Follow the configuration to enable the heading setup. Be aware that one receiver (reference receiver) has already be configured to receive corrections from a base station. Configure the ```enabled_sbp_messages``` on **uart1** instead. See the [documentation](https://support.swiftnav.com/customer/en/portal/articles/2805901-piksi-multi---heading) details.

5. Enable the [Ethernet Configuration](https://support.swiftnav.com/customer/en/portal/articles/2740815-using-ethernet-on-piksi-multi-and-duro) on reference and attitude receivers. Set `reference receiver ip_address to 192.168.0.222` and `attitude receiver ip_address to 192.168.0.223`

- Drivers configuration to be performed on the embedded device.

6. The ROS node reads SBP (Swift Navigation Binary Protocol) messages, a fast, simple, and minimal overhead binary protocol for communicating with Swift Navigation devices. 

**WARNING**: install __ONLY ONE__ version of SBP library, depending of which Hardware version you are using.

The following code will automatically download the required version of libsbp and install it in the default folder `/usr/local/lib/python2.7/dist-packages/sbp-<SBP_LIB_VERSION>-py2.7.egg/sbp/`.

```
# Execute this line in the package folder 'ethz_piksi_ros/piksi_multi_rtk_ros'
source install/install_piksi_multi.sh
```

7. To configure the ENU results from the ROS driver, fill the `enu_origin.yaml` on the package folder `er_localisation/cfg` with the same coordinates of the base station from step 3.

### Xsense
8. Install the MTi USB Serial Driver

	```
	$ git clone https://github.com/xsens/xsens_mt.git
	$ cd ~/xsens_mt
	$ make
	$ sudo modprobe usbserial
	$ sudo insmod ./xsens_mt.ko
	```

9. Install gps_common or gps_umd as available based on the ROS distributable

	```
	$ sudo apt-get install ros-kinetic-gps-umd

	```
	or
	```
	$ sudo apt-get install ros-kinetic-gps-common
	```
### Robot_localization package

10. Install the robot localization package

	```
	sudo apt-get install ros-$ROS_DISTRO-robot-localization
	```

## Usage

1. Test the ROS node on the reference receiver. The following line will launch the configuration over TCP/IP. Check that observations are received on the Swift Console and also the [published topics](https://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_rtk_ros#advertised-topics) from the driver   

	```
	$ roslaunch er_localisation piksi_multi_rover_reference.launch
	```

2. Test the ROS node on the attitude receiver. Check again observations and topics specially the `/piksi_attitude/baseline_heading`

	```
	$ roslaunch er_localisation piksi_multi_rover_attitude.launch
	```

3. Test the xsense node and check the publised topics.

	```
	$ roslaunch er_localisation xsens.launch
	```

4. The complete launch can be used to include sensor drivers and robot localization node to estimate the robot's pose in real time.

	```
	$ roslaunch er_localisation er_localisation_rover.launch
	```

## Earth Rover Localisation

The package includes .bag example files of recorded tracks to run robot localization and tune the EKF params if necessary.
The following launch file reproduces a bag file and applies the robot localization to adquire the pose estimation of the rover.

```
$ roslaunch er_localisation er_localisation_player.launch
```

### Input

The main published topics are:
- `/mti/sensor/imu`: A [sensor_msgs/Imu.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) with the imu data
- `/heading`: A [sensor_msgs/Imu.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) which reports the vehicle heading reported by the baseline between reference and attitude receiver
- `/piksi_receiver/navsatfix_best_fix`: A [sensor_msgs/NavSatFix.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html) that contains WGS 84 coordinates with best available fix at the moment (either RTK or SBAS)

### Nodes
Used nodes on the architecture
- `heading_listener`: In charge of traducing the vehicle heading from the baseline into ROS-compliant message to use in robot localization node
- `navsat_transform`: Takes as input the GPS data and produces an odometry message in coordinates that are consistent with the robot’s world frame. More info about navsat_transform_node on [Documentation](http://docs.ros.org/kinetic/api/robot_localization/html/navsat_transform_node.html).
- `ekf_localization`: The node is an implementation of an [Extended Kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter). It uses an omnidirectional motion model to project the state forward in time, and corrects that projected estimate using perceived sensor data. Detailed information on the [Documentation](http://docs.ros.org/kinetic/api/robot_localization/html/navsat_transform_node.html)	 page.

### Outputs

- `/odometry/gps`: A [nav_msgs/Odometry.msg](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html) message containing the GPS coordinates of your robot, transformed into its world coordinate frame.
- `gps/filtered`: A [sensor_msgs/NavSatFix.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html) message containing your robot’s world frame position, transformed into GPS coordinates.
- `/odometry/filtered/global`: A [nav_msgs/Odometry.msg](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html) message with the robot's pose estimation in its world frame.

### Mapviz 

You can install mapviz using apt-get from the ROS apt repository:

```
$ sudo apt-get install ros-$ROS_DISTRO-mapviz ros-$ROS_DISTRO-mapviz-plugins ros-$ROS_DISTRO-tile-map ros-$ROS_DISTRO-multires-image
```

Go to this [tutorial](https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite) to enable ROS Offline Google Maps for MapViz

- The following launch configures the vizualization tool 

	```
	$ roslaunch er_localisation localizacion_earthrover_viztools.launch
	```

Go to `File -> Open Config` on the top bar and upload the configuration file from `er_localisation/mapviz_config`