# Software

# Robot Localization

[Robot Localization](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html) is a collection of state estimation nodes, each of which is an implementation of a nonlinear state estimator for robots moving in 3D space. It contains two state estimation nodes, `ekf_localization_node` and `ukf_localization_node`. In addition, robot_localization provides `navsat_transform_node`, which aids in the integration of GPS data. 

The filter will not begin computation until it receives at least one message from one of the inputs.

Since the GPS data is subject to discrete discontinuities (“jumps”), it is likely to be unfit for use by navigation modules. If you want to fuse data from a GPS into your position estimate, one potential solution is to do the following:

1. **LOCAL EKF**: Run one instance of a `robot_localization` state estimation node that fuses only continuous data, such as odometry and IMU data. Set `world_frame = odom_frame` and execute local path plans and motions in this frame.

    Publishes local odometry and `odom→ugv_base_link` transform.

2. **GLOBAL EKF**: Run another instance of a `robot_localization` state estimation node that fuses all sources of data, including the GPS. Set  `world_frame = odom_frame` . Use the `navsat_transform_node` to transform the GPS data into an odometry message.

    Publishes `odometry/filtered` and `odom→base_link` transform. 

![Software%20e1211e53ec44473f88fea651cf72f568/LocalizationSetup.png](Software%20e1211e53ec44473f88fea651cf72f568/LocalizationSetup.png)

We have 2 separate trees, one for scouting and one for navigation. We unify them a posteriori.

The documentation recommends to set `world_frame = odom_frame` in the local EKF so it outputs `odom→base_link` and set `world_frame = map_frame` in the Global EKF so it outputs `map→odom` . But in this case, both `odom` and `map` are "world frames" and will diverge as you move.

See [ROSCon](https://vimeo.com/142624091) 2015 talk for useful explanatory video.

## Navsat Transform

This [node](http://docs.ros.org/en/noetic/api/robot_localization/html/navsat_transform_node.html) takes GPS data and produces an odometry message in coordinates that are consistent with your robot’s world frame. This value can be directly fused into your state estimate (EKF).

The `navsat_transform_node` also subscribes to two other topics: 

- IMU: GPS does not contain heading information and so the node needs to get that from the IMU in order to determine the transformation between UTM and map.
- Odometry output from the global EKF: The node only needs this data until it gets a first GPS fix.

The circular subscription (navsat subscribes to the odometry output of the EKF, and the EKF subscribes to the odometry output of navsat) is done because we need to know how far we have traveled from the start until we get a first GPS fix. If you don’t move the robot at all before you get a valid fix, you don't need an odometry source.

Once `navsat_transform_node` has the required data, it generates the required transform, and starts spitting out, for each GPS message, a pose that is consistent with your map/global frame EKF. So that data is then fed *back* to the map/global EKF. At this point, it [no longer needs IMU data](https://github.com/cra-ros-pkg/robot_localization/blob/a53709f364b6a88516632083387cb59c2bdd17bb/src/navsat_transform.cpp#L193).

If we use a manual datum, the `odom→utm` static transform is done at the start and is independent of external topics. The UTM grid assumes that the X-axis faces east, the Y-axis faces (true) north, and the Z-axis points up out of the ground (REP-105). Adding a yaw angle $\theta$ to the datum parameter will rotate the odom frame with the specified angle with respect to the UTM grid.

# Drivers

[Bibliography](https://www.notion.so/80de4e8282524447b4533cd088f59d10)

# Datum Convergece

[Useful resources](https://www.notion.so/f3636b096f4a4ec3bb97da6caf89d1b9)