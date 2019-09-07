title: robot_localization drifts when gps is missing (with RTK GPS, heading and IMU)
tags: 1.ros robot_localization gps+imu+heading odometry_drift kinetic
question:

Hi everyone,

After searching on the forum, I haven't found any answer that solved this issue so I hope it is not a duplicate.
The system we are using is: an outdoors robot that performs a high precision mapping of objects detected by a camera. To this objective the robot computes the localization using the following sensors:

1. GPS RTK:
- topic: `/piksi_receiver/navsatfix_rtk_fix`
- msg_type: sensor_msgs/NavSatFix
2. Heading RTK (in fact, there are 2 GPS antennas, and their drivers compute a high accurate heading using their position)
- topic: `/heading` (the driver sends a custom message in degrees and True north referenced, then, we transform it to ENU, so it complies with the required orientation specified by the robot_localization).
- msg_type: sensor_msgs/Imu
3. Imu
- topic: `/imu0` (the imu has a magnetometer, but it's magnetic field is highly affected by the system, so we are using it as a VRU device and we are resetting the `orientation.yaw` with the known `/heading`, so it complies with the required ENU orientation).
- msg_type: sensor_msgs/Imu

Covariances of all sensors are set accordingly to their datasheets.

**Problem explanation**

While there is RTK quality of the gps everything works smoothly, however, when there's a loss in the quality of the gps, the position it provides has an important offset. Hence, we decided to only use the gps position when the signal has RTK quality. And we though that the ekf would fill in the missing data with the heading and the imu.
However, as it can be seen in the images, while the heading (light blue arrow) keeps the correct orientation, the base_link has a drifting to the right over time. We tried different configurations of the robot_localization but all results were similar.

Has anyone an idea of what could be happening? Is there something wrong in the configuration of the robot localization? Are we missing something?

Thanks in advanced for your help!

**Images**

[Images of the results commented hereafter](https://drive.google.com/drive/folders/18jEivyTIwyzOEoKs3JVRJOW_H7S4ONI0?usp=sharing) (link to image as I don't have rights to upload them). Legend:

- Bright green points: rtk gps points (in WSG84)
- Grey points: bad gps quality (in WSG84)
- Green, Red and Blue lines: are the frames tracking for Reference (gps1), Attitude (gps2 for heading computing) and Robot base_link (odometry of frames in `/odom`)
- Light blue arrow:  is the odometry of the robot with heading.


**robot_localization configuration**

All transforms are defined in a URDF except by the transform from `map` to `odom` that is specified as a static transform in the roslaunch of the localization node (see below).

`ekf_imu_heading_localization.yaml`

   frequency: 60
   sensor_timeout: 0.1
   two_d_mode: false
   transform_time_offset: 0.0
   transform_timeout: 0.0
   print_diagnostics: true
   debug: false
   debug_out_file: /path/to/debug/file.txt
   publish_tf: true
   publish_acceleration: false

   map_frame: map              # Defaults to "map" if unspecified
   odom_frame: odom            # Defaults to "odom" if unspecified
   base_link_frame: base_link  # Defaults to "base_link" if unspecified
   world_frame: odom           # Defaults to the value of odom_frame if unspecified

   predict_to_current_time: true

   odom0: /odometry/gps
   odom0_config: [true,  true,  true,
                  false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, false]
   odom0_queue_size: 0
   odom0_nodelay: false
   odom0_differential: false
   odom0_relative: false

   imu0: /imu0
   imu0_config: [false, false, false,
                 true,  true,  false,
                 false, false, false,
                 true,  true,  true,
                 true, false, false]
   imu0_nodelay: false
   imu0_differential: false
   imu0_relative: false
   imu0_queue_size: 5
   imu0_remove_gravitational_acceleration: true

   imu1: /heading
   imu1_config: [false, false, false,
                 false, false, true,
                 false, false, false,
                 false, false, false,
                 false, false, false]
   imu1_nodelay: false
   imu1_differential: false
   imu1_relative: false
   imu1_queue_size: 0
   imu1_remove_gravitational_acceleration: true

   use_control: false
   stamped_control: false
   control_timeout: 0.2
   control_config: [true, false, false, false, false, true]
   acceleration_limits: [1.3, 0.0, 0.0, 0.0, 0.0, 3.4]
   deceleration_limits: [1.3, 0.0, 0.0, 0.0, 0.0, 4.5]
   acceleration_gains: [0.8, 0.0, 0.0, 0.0, 0.0, 0.9]
   deceleration_gains: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0]

   process_noise_covariance: [0.05,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                              0,0.05,0,0,0,0,0,0,0,0,0,0,0,0,0,
                              0,0,0.06,0,0,0,0,0,0,0,0,0,0,0,0,
                              0,0,0,0.03,0,0,0,0,0,0,0,0,0,0,0,
                              0,0,0,0,0.03,0,0,0,0,0,0,0,0,0,0,
                              0,0,0,0,0,0.06,0,0,0,0,0,0,0,0,0,
                              0,0,0,0,0,0,0.025,0,0,0,0,0,0,0,0,
                              0,0,0,0,0,0,0,0.025,0,0,0,0,0,0,0,
                              0,0,0,0,0,0,0,0,0.04,0,0,0,0,0,0,
                              0,0,0,0,0,0,0,0,0,0.01,0,0,0,0,0,
                              0,0,0,0,0,0,0,0,0,0,0.01,0,0,0,0,
                              0,0,0,0,0,0,0,0,0,0,0,0.02,0,0,0,
                              0,0,0,0,0,0,0,0,0,0,0,0,0.01,0,0,
                              0,0,0,0,0,0,0,0,0,0,0,0,0,0.01,0,
                              0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.015]

   initial_estimate_covariance: [1e-6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                 0,1e-6,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                 0,0,1e-9,0,0,0,0,0,0,0,0,0,0,0,0,
                                 0,0,0,1e-9,0,0,0,0,0,0,0,0,0,0,0,
                                 0,0,0,0,1e-9,0,0,0,0,0,0,0,0,0,0,
                                 0,0,0,0,0,1e-3,0,0,0,0,0,0,0,0,0,
                                 0,0,0,0,0,0,1e-9,0,0,0,0,0,0,0,0,
                                 0,0,0,0,0,0,0,1e-9,0,0,0,0,0,0,0,
                                 0,0,0,0,0,0,0,0,1e-9,0,0,0,0,0,0,
                                 0,0,0,0,0,0,0,0,0,1e-9,0,0,0,0,0,
                                 0,0,0,0,0,0,0,0,0,0,1e-9,0,0,0,0,
                                 0,0,0,0,0,0,0,0,0,0,0,1e-9,0,0,0,
                                 0,0,0,0,0,0,0,0,0,0,0,0,1e-9,0,0,
                                 0,0,0,0,0,0,0,0,0,0,0,0,0,1e-9,0,
                                 0,0,0,0,0,0,0,0,0,0,0,0,0,0,1e-9]

`er_localization_node.launch`

   <launch>
     <!-- Frames  -->
     <node pkg="tf2_ros" type="static_transform_publisher" name="map_odom" args="0 0 0 0 0 0 map odom" />
     <node pkg="tf2_ros" type="static_transform_publisher" name="bl_gps_receiver" args="0 -0.95 0 0 0 0 base_link gps_receiver" />
     <node pkg="tf2_ros" type="static_transform_publisher" name="bl_imu" args="-0.149 0.00282 -0.09554 3.1415926535897931 0 0 base_link xsens" />

     <!-- EKF node -->
     <node pkg="robot_localization" type="ekf_localization_node" name="ekf_localization">
       <rosparam command="load" file="$(find earth_rover_localization)/cfg/ekf_imu_heading_localization.yaml" />
       <remap from="/odometry/filtered" to="/odometry/filtered/global"/>
     </node>

    <!-- Navsat_transform -->
    <node pkg="robot_localization" type="navsat_transform_node" name="navsat_transform" respawn="true" output="screen">
       <param name="frequency" value="30"/>
       <param name="delay" value="3.0"/>
       <param name="magnetic_declination_radians" value="0.0"/>
       <param name="yaw_offset" value="0"/>
       <param name="zero_altitude" value="false"/>
       <param name="broadcast_utm_transform" value="true"/>
       <param name="publish_filtered_gps" value="true"/>
       <param name="use_odometry_yaw" value="false"/>
       <param name="wait_for_datum" value="true"/>
       <rosparam param="datum">[53.433998, -0.908099, 0.0]</rosparam>
       <remap from="/odometry/filtered" to="/odometry/filtered/global"/>
       <remap from="/gps/fix" to="/piksi_receiver/navsatfix_rtk_fix"/>
     </node>
   </launch>
