# earth_rover_localization [![Build Status](https://travis-ci.com/earthrover/OpenER.svg?branch=master)](https://travis-ci.com/earthrover/OpenER)

This repository contains ROS drivers, tools, launch files, and documents to configure the EKF robot_localization for the Earth Rover Agribot.

![](https://github.com/earthrover/er_localisation/blob/master/earth_rover_localization/docs/OpenER.jpg)

Overview
------
- [earth_rover_localization](https://github.com/earthrover/er_localisation/tree/master/er_localisation): ROS package to configure the EKF of the robot_localization package. Uses sensor fusion of GPS [Piksy Multi](https://www.swiftnav.com/piksi-multi) and IMU [MTi-3 AHRS](https://www.xsens.com/products/mti-1-series/)
- [ethz_piksi_ros](https://github.com/ethz-asl/ethz_piksi_ros/tree/767b0192be2f1a3e5434bcf6ddf33bb3dbd96c4f): Repository that contains ROS driver and utilities for Piksi RTK receiver device.
- [xsens_mti_ros_node](https://github.com/xsens/xsens_mti_ros_node): ROS driver for third and fourth generation of Xsens IMU devices.

License
-------
The source code is released under a [BSD 3-Clause license](https://github.com/earthrover/er_localisation/blob/master/LICENSE.md).

Bugs & Feature Requests
-------
Please report bugs and request features using the [Issue Tracker](https://github.com/earthrover/er_localisation/issues).

Acknowledgement
-------
Earth Rover ROS – The Open-Source Agribot (http://open.earthrover.cc/)

Andres Palomino apalomino(at)edgebrain.io
Ricard Pardell ricard(at)earthrover.cc

Based on robot_localization from Tom Moore.
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287.
