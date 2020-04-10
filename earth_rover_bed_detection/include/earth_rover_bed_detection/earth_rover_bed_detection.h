/*************************************************************
 Copyright (C) 2019 Earth Rover Limited. All rights reserved.
*************************************************************/

// C++ standard libraries
#include <string>
#include <climits>

// ROS libraries
#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

namespace nm = nav_msgs;

/**
 * @class PlantUTM2GPS
 * @brief This class implements the node converting the plant's origin from utm to gps.
 */
class BedDetection {
  public:
    /**
     * Class constructor
     * @param node_handle
     * @param private_node_handle
     */
    BedDetection(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle);

    /**
     *  Class destructor
     */
    ~BedDetection() = default;

    bool ok();

    void odom_callback(const nm::OdometryPtr& odom_sub);

    bool is_straight();


    /**
     * Initializes the ROS node (creates variables, publishers and subscriebrs).
     */
    void init();

 private:
    // ros node handles
    ros::NodeHandle _nh;          ///< node handle
    ros::NodeHandle _pnh;         ///< private node handle
    // odometry subscriber
    ros::Subscriber _sub_odom;

    bool _is_init;

    nm::Odometry _odom_msg;

    // param variables

};
