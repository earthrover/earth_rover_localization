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
  std::string itopic_odom  = "/ugv_11/earth_rover_ugv_diff/odom";           // input topic for robot odometry
  _sub_odom = _nh.subscribe(itopic_odom, 1, &BD::odom_callback, this);

}

bool BD::ok()
{
  return _is_init;
}

void BD::odom_callback(const nm::OdometryPtr& odom_sub)
{
  _odom_msg = *odom_sub;
  BD::is_straight();
}

bool BD::is_straight()
{
  if (_odom_msg.twist.twist.angular.z > 0.2)
  {
    ROS_INFO_STREAM("Patito false");
    return false;
  }
  else
  {
    ROS_INFO_STREAM("Patito true");
    return true;
  }
}
