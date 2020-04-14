/*************************************************************
 Copyright (C) 2019 Earth Rover Limited. All rights reserved.
*************************************************************/

#include "earth_rover_bed_detection/earth_rover_bed_detection.h"

using BD = BedDetection;

BD::BedDetection(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle) :
  _nh(node_handle), _pnh(private_node_handle)
{
  _is_init = true;
  this->init();
}

void BD::init()
{
  std::string itopic_odom;  //= "/ugv_11/earth_rover_ugv_diff/odom";  // input topic for robot odometry

  // Obtain ros parameters
  if (ros::param::get("~itopic_odom", itopic_odom)){}
  else
  {
    ros::shutdown();
  }
  if (ros::param::get("~threshold_time", _threshold_time)){}
  else
  {
    ros::shutdown();
  }
  if (ros::param::get("~twist_threshold", _twist_threshold)){}
  else
  {
    ros::shutdown();
  }
  // Init subscribers
  _sub_odom = _nh.subscribe(itopic_odom, 1, &BD::odom_callback, this);

  // Init publishers
  _pub_bed_detection = _nh.advertise<std_msgs::Bool>("is_straight", 1000);
}

bool BD::ok()
{
  return _is_init;
}

void BD::odom_callback(const nm::OdometryPtr& odom_sub)
{
  _odom_msg = *odom_sub;
  bool isstraight = BD::is_straight();
  std_msgs::Bool x;
  x.data = isstraight;
  _pub_bed_detection.publish(x);
}


// bool BD::is_straight()
// {
//   if (_odom_msg.twist.twist.angular.z > 0.2)
//   {
//     ROS_INFO_STREAM("false");
//     return false;
//   }
//   else
//   {
//     ROS_INFO_STREAM("true");
//     return true;
//   }
// }

bool BD::is_straight()
{
  if (_odom_msg.twist.twist.angular.z > _twist_threshold || _odom_msg.twist.twist.angular.z < -_twist_threshold)
  {
    if (_first_time == true)
    {
      _tp_1 = ros::Time::now();
      _first_time = false;
    }
    ros::Time tp_2 = ros::Time::now();
    double track_time = (tp_2 - _tp_1).toSec();
    // ROS_INFO_STREAM(track_time);
    if (track_time > _threshold_time)
    {
      ROS_INFO_STREAM("Not Straight");
      return false;
    }
  }
  else
  {
    _first_time = true;
    ROS_INFO_STREAM("Straight");
    return true;
  }
}
