# Robot Localization

[Robot Localization](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html) is a collection of state estimation nodes, each of which is an implementation of a nonlinear state estimator for robots moving in 3D space. It contains two state estimation nodes, `ekf_localization_node` and `ukf_localization_node`. In addition, robot_localization provides `navsat_transform_node`, which aids in the integration of GPS data.

The filter will not begin computation until it receives at least one message from one of the inputs.

Since the GPS data is subject to discrete discontinuities (“jumps”), it is likely to be unfit for use by navigation modules. If you want to fuse data from a GPS into your position estimate, one potential solution is to do the following:

1. **LOCAL EKF**: Run one instance of a `robot_localization` state estimation node that fuses only continuous data, such as odometry and IMU data. Set `world_frame = odom_frame` and execute local path plans and motions in this frame.

    Publishes local odometry and `odom→ugv_base_link` transform.

2. **GLOBAL EKF**: Run another instance of a `robot_localization` state estimation node that fuses all sources of data, including the GPS. Set  `world_frame = odom_frame` . Use the `navsat_transform_node` to transform the GPS data into an odometry message.

    Publishes `odometry/filtered` and `odom→scouting_base_link` transform.

![](https://github.com/earthrover/er_localisation/blob/master/earth_rover_localization/docs/imgs/localization_setup.png)

We have 2 separate trees, one for scouting and one for navigation. We unify them a posteriori.

The documentation recommends to set `world_frame = odom_frame` in the local EKF so it outputs `odom→base_link` and set `world_frame = map_frame` in the Global EKF so it outputs `map→odom` . But in this case, both `odom` and `map` are "world frames" and will diverge as you move.

See [ROSCon](https://vimeo.com/142624091) 2015 talk for useful explanatory video.

## Navsat Transform

The [navsat](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) node takes GPS data and produces an odometry message in coordinates that are consistent with your robot’s world frame. This value can be directly fused into your state estimate (EKF).

The `navsat_transform_node` also subscribes to two other topics:

- IMU: GPS does not contain heading information and so the node needs to get that from the IMU in order to determine the transformation between UTM and map.
- Odometry output from the global EKF: The node only needs this data until it gets a first GPS fix.

The circular subscription (navsat subscribes to the odometry output of the EKF, and the EKF subscribes to the odometry output of navsat) is done because we need to know how far we have traveled from the start until we get a first GPS fix. If you don’t move the robot at all before you get a valid fix, you don't need an odometry source.

Once `navsat_transform_node` has the necessary data, it generates the required transform, and starts spitting out, for each GPS message, a pose that is consistent with your map/global frame EKF. So that data is then fed back to the map/global EKF. At this point, it [no longer needs IMU data](https://github.com/cra-ros-pkg/robot_localization/blob/a53709f364b6a88516632083387cb59c2bdd17bb/src/navsat_transform.cpp#L193).

If we use a manual datum, the `world→utm` static transform is done at the start and is independent of external topics. In our case, `world=odom` so we will have `odom→utm`. The UTM grid assumes that the X-axis faces east, the Y-axis faces (true) north, and the Z-axis points up out of the ground ([REP-105](https://www.ros.org/reps/rep-0105.html)). Adding a yaw angle to the datum parameter will rotate the odom frame with the specified angle with respect to the UTM grid.

The workflow of the navsat node is shown below.

![](https://github.com/earthrover/er_localisation/blob/master/earth_rover_localization/docs/imgs/navsat_workflow.png)

Upon start, we call the `datumCallback` and set the GPS and odometry transforms as well as correct our orientation with the IMU. For our use case where we use manual datum, the initial odometry position is (0,0,0) and the initial corrected orientation is RPY(0,0,0). Next, we perform a periodic update. The first time this function gets called, the `computeTransform` function is executed and we calculate the static UTM to odom transform. After that, we only call `prepareGpsOdometry` and `prepareFilteredGps` (optionally) inside the periodic update.
- In `prepareGpsOdometry` we feed a UTM GPS measurement processed by the `gpsFixCallback`, which listens to GPS data and transforms it from LL (latitude and longitude) to UTM. By calling `utmToMap` we obtain the odom pose of the GPS receiver, which is then transformed using the `getRobotOriginWorldPose` function to calculate the odom pose of the robot base_link. Finally, we publish the odometry message that will feed the global EKF.
- If `publish_gps=true`, the `prepareFilteredGps` function gets called in order to transform the odometry message published by the global EKF to a LL GPS message.

**Our system assumes that we require GPS RTK fix data in order to start a scouting session**.

### Bibliography

1. [Outdoor Global Localization](https://www.robotandchisel.com/2020/05/01/outdoor-navigation/)
2. [Navsat Documentation](https://github.com/cra-ros-pkg/robot_localization/issues/550)
3. [RosAnswers RL Setup](https://answers.ros.org/question/202117/integrate-a-gps-sensor-with-robot_localization-and-use-move_base-node/)
