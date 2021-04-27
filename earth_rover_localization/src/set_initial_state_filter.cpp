/**
 * @file set_initial_state_EKF.cpp
 * @brief This file takes a GPS measurement and transforms it into odom frame to set the initial_state parameter for the EKF filter
          Waits until we have RTK fix to use the GPS data and save the initial state parameter
 * @author Xavier Ruiz (xavier.ruiz@earthrover.farm)
 */

 #include <ros/ros.h>
 #include <std_msgs/Float64.h>
 #include <geometry_msgs/Vector3.h>
 #include "robot_localization/navsat_conversions.h"
 #include <tf/transform_datatypes.h>
 #include "robot_localization/ros_filter_utilities.h"
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 #include <nav_msgs/Odometry.h>
 #include <sensor_msgs/Imu.h>
 #include <sensor_msgs/NavSatFix.h>
 #include <Eigen/Dense>
 #include <string>
 #include <iostream>
 #include <fstream>
 #include <tf2/LinearMath/Transform.h>
 #include <tf2_ros/static_transform_broadcaster.h>
 #include <tf2_ros/buffer.h>
 #include <tf2_ros/transform_listener.h>
 #include "ros/package.h"

 class SetInitialStateFilter
 {
 public:
     SetInitialStateFilter(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle)
     : nh_(node_handle), pnh_(private_node_handle)
     {
         this->init();
     }
     ~SetInitialStateFilter() = default;

     void init();
     void headingCb(const sensor_msgs::ImuConstPtr& imu_msg);
     void gpsCallback(const sensor_msgs::NavSatFixConstPtr& msg);
     nav_msgs::Odometry utmToMap(const tf2::Transform& utm_pose, const ros::Time &transform_time) const;
     void gpsToRobotPose(const tf2::Transform &gps_odom_pose, tf2::Transform &robot_odom_pose, const ros::Time &transform_time);

     // Initialize Variables
     int gps_count_{0};
     int N_gps_{1};
     int heading_count_{0};
     int N_heading_{1};
     bool zero_altitude_{false};
     bool rtk_fix_ {false};
     std::string base_link_frame_id_{"scouting_wrt_ground_link"};
     std::string world_frame_id_{"odom"};
     std::string gps_frame_id_{""};
     std::string path_;
     Eigen::MatrixXd utm_covariance_;
     ros::Duration transform_timeout_{ros::Duration(0)};
     tf::Quaternion robot_quat_;
     tf2::Quaternion robot_orientation_;
     tf2::Transform utm_pose_;
     tf2_ros::Buffer tf_buffer_;
     tf2_ros::TransformListener tf_listener_{tf_buffer_};
     sensor_msgs::NavSatFix gps_msg_avg_;
     sensor_msgs::Imu imu_msg_avg_;

     // public and private ros node handle
     ros::NodeHandle nh_;
     ros::NodeHandle pnh_;

     // Subscribers and publishers
     ros::Subscriber sub_heading_;
     ros::Subscriber sub_gps_;
 };

 void SetInitialStateFilter::init()
 {
     ROS_INFO("Numb GPS: %d", N_gps_);
     ROS_INFO("Numb heading: %d", N_heading_);
     ROS_INFO("Loading params...");
     // Load params
     nh_.getParam("/navsat_transform/zero_altitude", zero_altitude_);
     nh_.getParam("/localization_init/N_gps", N_gps_);
     nh_.getParam("/localization_init/N_heading", N_heading_);

     sub_heading_ = nh_.subscribe("/heading", 1000, &SetInitialStateFilter::headingCb, this);
     sub_gps_ = nh_.subscribe("/piksi_receiver/navsatfix_best_fix",1000, &SetInitialStateFilter::gpsCallback, this);
     ROS_INFO("Numb GPS: %d", N_gps_);
     ROS_INFO("Numb heading: %d", N_heading_);
     path_ = ros::package::getPath("earth_rover_localization")+"/cfg/initial_state.yaml";
     ROS_INFO("Path: %s", path_.c_str());
 }

 void SetInitialStateFilter::headingCb(const sensor_msgs::ImuConstPtr& imu_msg)
 {
    // Only average heading when we have rtk fix gps (heading comes from fps)
    if (rtk_fix_){
       heading_count_ ++;
       if (heading_count_==1){
         imu_msg_avg_ = *imu_msg;
       }
       else{
         imu_msg_avg_.orientation.x += imu_msg->orientation.x;
         imu_msg_avg_.orientation.y += imu_msg->orientation.y;
         imu_msg_avg_.orientation.z += imu_msg->orientation.z;
         imu_msg_avg_.orientation.w += imu_msg->orientation.w;
         imu_msg_avg_.header.stamp = imu_msg->header.stamp;
       }
    }
 }

 void SetInitialStateFilter::gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg)
 {
    // Wait until we have RTK fix
    if (gps_msg->status.status==2 && gps_msg->position_covariance[0]<=0.0049){
      rtk_fix_ = true;
      gps_count_ ++;
      if (gps_count_==1){
        gps_msg_avg_ = *gps_msg;
        gps_frame_id_ = gps_msg->header.frame_id;
      }
      else{
        gps_msg_avg_.latitude += gps_msg->latitude;
        gps_msg_avg_.longitude += gps_msg->longitude;
        gps_msg_avg_.altitude += gps_msg->altitude;
        gps_msg_avg_.header.stamp = gps_msg->header.stamp;
      }
    }
    else{
      rtk_fix_ = false;
      ROS_WARN("Waiting for RTK fix to initialize localization...");
    }

    if (gps_count_ >= N_gps_ && heading_count_ >= N_heading_)
    {
        // Calculate GPS and IMU average values
        gps_msg_avg_.latitude /= gps_count_;
        gps_msg_avg_.longitude /= gps_count_;
        gps_msg_avg_.altitude /= gps_count_;
        imu_msg_avg_.orientation.x /= heading_count_;
        imu_msg_avg_.orientation.y /= heading_count_;
        imu_msg_avg_.orientation.z /= heading_count_;
        imu_msg_avg_.orientation.w /= heading_count_;

        // Transform GPS LL data into UTM coordintes
        double utm_x = 0.0;
        double utm_y = 0.0;
        double utm_z = 0.0;
        std::string utm_zone_tmp;
        RobotLocalization::NavsatConversions::LLtoUTM(gps_msg_avg_.latitude, gps_msg_avg_.longitude, utm_y, utm_x, utm_zone_tmp);
        utm_pose_.setOrigin(tf2::Vector3(utm_x, utm_y, gps_msg_avg_.altitude));
        utm_covariance_.setZero();

        // Transform UTM pose into map frame
        nav_msgs::Odometry gps_odom;
        gps_odom = utmToMap(utm_pose_, gps_msg_avg_.header.stamp);
        tf2::Transform transformed_utm_gps;
        tf2::fromMsg(gps_odom.pose.pose, transformed_utm_gps);

        // Want the pose of the vehicle origin, not the GPS
        tf2::Transform transformed_utm_robot;
        tf2::fromMsg(imu_msg_avg_.orientation, robot_orientation_); // get robot orientation tf from heading avg
        gpsToRobotPose(transformed_utm_gps, transformed_utm_robot, gps_odom.header.stamp);

        // Write to yaml file initial state param
        std::ofstream myfile;
        nav_msgs::Odometry init_state{};
        // Get quat orientation to tf in order to get RPY
        tf::quaternionMsgToTF(imu_msg_avg_.orientation, robot_quat_);
        tf2::toMsg(transformed_utm_robot, init_state.pose.pose);
        double roll, pitch, yaw;
        tf::Matrix3x3(robot_quat_).getRPY(roll, pitch, yaw);
        myfile.open(path_);
        myfile << "initial_state: [" <<  init_state.pose.pose.position.x << ", ";
        myfile << init_state.pose.pose.position.y << ", ";
        myfile << init_state.pose.pose.position.z << ",\n";
        myfile << 0.0 << ", ";
        myfile << 0.0 << ", ";
        myfile << yaw << ", \n";
        myfile << 0.0 << ", " << 0.0 << ", " << 0.0 << ", \n";
        myfile << 0.0 << ", " << 0.0 << ", " << 0.0 << ", \n";
        myfile << 0.0 << ", " << 0.0 << ", " << 0.0 << "]";

        myfile.close();
        ROS_INFO("Loaded initial_state param into yaml file");
        ros::shutdown();
    }

 }

 nav_msgs::Odometry SetInitialStateFilter::utmToMap(const tf2::Transform& utm_pose, const ros::Time &transform_time) const
 {
   nav_msgs::Odometry gps_odom{};
   tf2::Transform transformed_utm_gps{};

   tf2::Transform utm_world_transform_;
   bool can_transform = RobotLocalization::RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                                world_frame_id_,
                                                                "utm",
                                                                transform_time,
                                                                ros::Duration(5),
                                                                utm_world_transform_);

   if (can_transform)
   {
      transformed_utm_gps.mult(utm_world_transform_, utm_pose);
      transformed_utm_gps.setRotation(tf2::Quaternion::getIdentity());
   }

   // Set header information stamp because we would like to know the robot's position at that timestamp
   gps_odom.header.frame_id = world_frame_id_;

   // Now fill out the message. Set the orientation to the identity.
   tf2::toMsg(transformed_utm_gps, gps_odom.pose.pose);
   gps_odom.pose.pose.position.z = (zero_altitude_ ? 0.0 : gps_odom.pose.pose.position.z);

   return gps_odom;
 }

 void SetInitialStateFilter::gpsToRobotPose(const tf2::Transform &gps_odom_pose,
                                               tf2::Transform &robot_odom_pose,
                                               const ros::Time &transform_time)
 {
     tf2::Transform gps_offset_rotated;
     bool can_transform = RobotLocalization::RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                                  base_link_frame_id_,
                                                                  gps_frame_id_,
                                                                  transform_time,
                                                                  transform_timeout_,
                                                                  gps_offset_rotated);

     if (can_transform)
     {
         gps_offset_rotated.setOrigin(tf2::quatRotate(robot_orientation_, gps_offset_rotated.getOrigin()));
         gps_offset_rotated.setRotation(tf2::Quaternion::getIdentity());
         robot_odom_pose = gps_offset_rotated.inverse() * gps_odom_pose;
     }
 }

 int main(int argc, char **argv)
 {
     ros::init(argc, argv, "set_initial_state_filter");
     ros::NodeHandle nh;
     ros::NodeHandle nh_private("~");
     SetInitialStateFilter node(nh, nh_private);
     ros::spin();

     return 0;
 }
