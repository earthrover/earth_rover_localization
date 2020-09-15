/*************************************************************
 Copyright (C) 2019 Earth Rover Limited. All rights reserved.
*************************************************************/

/**
 * @file bed_detection.cpp
 * @brief Class implementation of the Bed Detection, which estimates wether the robot is moving in a straight line or not.
 * @author Rodrigo Gonzalez (rodrigo.gonzalez@earthrover.farm)
 * @date Apr 2020
 */

#include "earth_rover_bed_detection/bed_detection.h"

using BD = BedDetection;


BD::BedDetection(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle) :
  _nh(node_handle), _pnh(private_node_handle)
{
  _is_init = true;
  this->init();
}


void BD::init()
{
  // topic names
  std::string itopic_odom = "odom";
  std::string otopic_is_straight = "is_straight";

  // Init variables
  _first_time = true;

  // Get ROS parameters
  std::string threshold_time = "/bed_detection/threshold_time";
  if( !_nh.getParam(threshold_time, _threshold_time) )
  {
    ROS_ERROR_STREAM("[MAPPING] Missing parameter: " << threshold_time);
    _threshold_time = 0.5;
  }
  ROS_INFO_STREAM("/threshold_time: " << _threshold_time);
  std::string threshold_twist = "/bed_detection/threshold_twist";
  if( !_nh.getParam(threshold_twist, _threshold_twist) )
  {
    ROS_ERROR_STREAM("[MAPPING] Missing parameter: " << threshold_twist);
    _threshold_twist = 0.2;
  }
  ROS_INFO_STREAM("/threshold_twist: " << _threshold_twist);

  // Init subscribers
  _sub_odom = _nh.subscribe(itopic_odom, 1, &BD::cb_is_straight, this);

  // Init publishers
  _pub_is_straight = _nh.advertise<std_msgs::Bool>(otopic_is_straight, 1000);

}


bool BD::ok()
{
  return _is_init;
}


void BD::cb_is_straight(const nm::OdometryPtr& odom)
{
  // get message data
  _odom = *odom;

  // process message
  std_msgs::Bool msg;
  msg.data = BD::is_straight();
  // msg.data = BD::is_straight_time();

  // publish result
  _pub_is_straight.publish(msg);

}


bool BD::is_straight()
{
  if( abs(_odom.twist.twist.angular.z) > _threshold_twist )
  {
    ROS_INFO_STREAM("false");
    return false;
  }
  else
  {
    ROS_INFO_STREAM("true");
    return true;
  }
}


bool BD::is_straight_time()
{
  if( abs(_odom.twist.twist.angular.z) > _threshold_twist )
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
