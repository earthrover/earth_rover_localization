/**
 * @file heading_listener.cpp
 * @brief This file includes a listener ROS node for the swiftnav heading message and publishes the ROS-compliant heading in a imu message.
 * @author Andres Palomino (apalomino@edgebrain.io)
 */


 #include <ros/ros.h>
 #include <std_msgs/String.h>
 #include <std_msgs/UInt32.h>
 #include <std_msgs/Float32.h>
 #include <std_msgs/Float64.h>
 #include <tf/transform_datatypes.h>
 #include <sensor_msgs/Imu.h>
 #include <piksi_rtk_msgs/BaselineHeading.h>

 # define M_PI           3.14159265358979323846  /* pi */
 # define sensor_yaw_deviation            0.002882263

 class HeadingListenerNode
 {
 public:
     HeadingListenerNode(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle)
     : _nh(node_handle), _pnh(private_node_handle)
     {
         this->init();
     }
     ~HeadingListenerNode() = default;

     void init();
     void cb_heading_ned2enu(const piksi_rtk_msgs::BaselineHeading::ConstPtr& msg);
     void cb_imu_vru2enu(const sensor_msgs::Imu::ConstPtr& msg);

     // Initialize variables
     // imu message
     sensor_msgs::Imu _imu;
     // reset values for imu
     bool _heading_new = false;
     double _heading0 = 0.0;
     double _imu0 = 0.0;
     // Offset angle between the heading frame and the imu frame
     double _yaw_hi = 180.0;

     // public ros node handle
     ros::NodeHandle _nh;
     // private ros node handle
     ros::NodeHandle _pnh;
     // Subscribers and publishers
     ros::Subscriber _sub_heading_ned;
     ros::Subscriber _sub_imu_vru;
     ros::Publisher  _pub_heading_enu;
     ros::Publisher  _pub_imu_enu;

 };

 void HeadingListenerNode::init()
 {
     _sub_heading_ned = _nh.subscribe("/piksi_attitude/baseline_heading", 1000, &HeadingListenerNode::cb_heading_ned2enu, this);
     _pub_heading_enu = _nh.advertise<sensor_msgs::Imu>("/heading", 1);

     _sub_imu_vru = _nh.subscribe("/mti/sensor/imu", 1000, &HeadingListenerNode::cb_imu_vru2enu, this);
     _pub_imu_enu = _nh.advertise<sensor_msgs::Imu>("/imu", 1);

     _imu.orientation_covariance[0] = (1 * M_PI/180.0)*(1 * M_PI/180.0);
     _imu.orientation_covariance[4] = _imu.orientation_covariance[0];
     _imu.orientation_covariance[8] = (9 * M_PI/180.0)*(9 * M_PI/180.0);
     _imu.angular_velocity_covariance[0] = (0.25 * M_PI/180.0)*(0.25 * M_PI/180.0);
     _imu.angular_velocity_covariance[4] = _imu.angular_velocity_covariance[0];
     _imu.angular_velocity_covariance[8] = _imu.angular_velocity_covariance[0];
     _imu.linear_acceleration_covariance[0] = 0.0004;
     _imu.linear_acceleration_covariance[4] = _imu.linear_acceleration_covariance[0];
     _imu.linear_acceleration_covariance[8] = _imu.linear_acceleration_covariance[0];
 }

 void HeadingListenerNode::cb_heading_ned2enu(const piksi_rtk_msgs::BaselineHeading::ConstPtr& msg)
 {
     // Variables
     std_msgs::UInt32 heading;
     std_msgs::Float32 heading_deg, heading_radians;
     sensor_msgs::Imu heading_rtk;

     // Get heading data (standard heading [mdeg])
     heading.data =  msg->heading;

     // Transform to ENU
     heading_deg.data = (float)heading.data/1000;
     heading_deg.data = 360 - heading_deg.data;    // Orientation increasing Counter clockwise
     heading_deg.data = heading_deg.data + 90;    // 0° pointing East
     if (heading_deg.data > 360.0)
         heading_deg.data = heading_deg.data - 360.0;
     heading_radians.data = heading_deg.data * M_PI/180.0;

     // Save heading in degrees
     _heading_new = true;
     _heading0 = heading_deg.data;

     // Debug heading readings
     ROS_DEBUG("Heading : [%d] deg ENU: [%f] rad ENU: [%f]", heading.data, heading_deg.data, heading_radians.data);

     // Create imu msg from heading RTK
     heading_rtk.header.stamp = msg->header.stamp;
     heading_rtk.header.frame_id = "gps_receiver";
     heading_rtk.orientation = tf::createQuaternionMsgFromYaw(heading_radians.data);  // Create this quaternion from yaw (in radians)
     // Calculated standard deviation for the sensor
     double standard_deviation = sensor_yaw_deviation  * M_PI/180.0;
     heading_rtk.orientation_covariance[8] = standard_deviation*standard_deviation;   //Update sensor covariance about z axe
     heading_rtk.linear_acceleration.x = 0;
     heading_rtk.linear_acceleration.y = 0;
     heading_rtk.linear_acceleration.z = 0;
     heading_rtk.linear_acceleration_covariance[0] = -1;
     heading_rtk.angular_velocity.x = 0;
     heading_rtk.angular_velocity.y = 0;
     heading_rtk.angular_velocity.z = 0;
     heading_rtk.angular_velocity_covariance[0] = -1;

     // Update last message stamp
     _pub_heading_enu.publish(heading_rtk);
 }

 void HeadingListenerNode::cb_imu_vru2enu(const sensor_msgs::Imu::ConstPtr& msg)
 {
     // Get message
     _imu.header = msg->header;
     _imu.angular_velocity = msg->angular_velocity;
     _imu.linear_acceleration = msg->linear_acceleration;

     // Get imu yaw in degrees [0,360)
     tf::Quaternion quat;
     double roll, pitch, yaw;
     tf::quaternionMsgToTF(msg->orientation, quat);
     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
     yaw = yaw < 0 ? (yaw * 180.0 / M_PI)+360 : (yaw * 180.0 / M_PI);

     // Set reference if needed
     if (_heading_new)
     {
       _imu0 = yaw;
       _heading_new = false;
     }

     // Compute corrected orientation
     double yaw_delta = (yaw - _imu0);
     if (yaw_delta > 180) { yaw_delta = yaw - 360 - _imu0; } // Shortest path difference
     if (yaw_delta < -180) { yaw_delta = yaw + 360 - _imu0; }
     yaw = _heading0 + _yaw_hi + yaw_delta;
     while (yaw > 360) { yaw -= 360; } // yaw between [0, 360)
     while (yaw < 0) { yaw += 360; }
     yaw *= M_PI/180.0;
     _imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

     // Publish corrected imu message
     _pub_imu_enu.publish(_imu);
 }

 int main(int argc, char **argv)
 {
     ros::init(argc, argv, "heading_listener");
     ros::NodeHandle nh;
     ros::NodeHandle nh_private("~");
     HeadingListenerNode node(nh, nh_private);
     ros::spin();

     return 0;
 }
