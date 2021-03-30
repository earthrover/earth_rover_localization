/*************************************************************
 Copyright (C) 2019 Earth Rover Limited. All rights reserved.
*************************************************************/

/**
 * @file bed_detection.h
 * @brief Header for theBed Detection Class.
 * @author Rodrigo Gonzalez (rodrigo.gonzalez@earthrover.farm)
 * @date Apr 2020
 */

#ifndef __BED_DETECTION_CLASS_H__
#define __BED_DETECTION_CLASS_H__

// C++ standard libraries
#include <string>
#include <climits>
#include <algorithm>
#include <chrono>

// ROS libraries
#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

namespace nm = nav_msgs;

/**
 * @class BedDetection
 * @brief This class implements the detection of a bed given an odometry.
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

    /**
     * Initializes the ROS node (creates variables, publishers and subscriebrs).
     */
    void init();

    /**
     * Checks if the node is properly initialized.
     * @return 'true' if the initialization is correct.
     */
    bool ok();


    void cb_is_straight(const nm::OdometryPtr& odom_sub);

    bool is_straight();

    bool is_straight_time();

 private:
    // ros node handles
    ros::NodeHandle _nh;          ///< node handle
    ros::NodeHandle _pnh;         ///< private node handle

    // Subscribers
    ros::Subscriber _sub_odom;

    // Publishers
    ros::Publisher _pub_is_straight;

    // System vars
    bool _is_init;                ///< states if system is init
    bool _first_time;
    double _threshold_time;
    double _threshold_twist;
    ros::Time _tp_1;

    // ROS messages
    nm::Odometry _odom;

};

#endif  // __BED_DETECTION_CLASS_H__
