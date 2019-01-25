/**
 * @file heading_listener.cpp
 * @brief This file includes a listener ROS node for the swiftnav heading message and publishes the ROS-compliant heading in a imu message.
 * @author Andres Palomino (apalomino@edgebrain.io)
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "piksi_rtk_msgs/BaselineHeading.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

# define M_PI           3.14159265358979323846  /* pi */
# define sensor_yaw_deviation            0.002882263

// Initialize variables
sensor_msgs::Imu heading_rtk;
std_msgs::Float32 heading_radians;

/**
* @brief Callback function to transform heading from RTK into ROS-compliant Imu message
* @param Piksy baselineheading message
*
******************************************************************/
void heading_callback(const piksi_rtk_msgs::BaselineHeading::ConstPtr& msg)
{
    std_msgs::UInt32 heading;
    std_msgs::Float32 heading_deg;
    // Get heading data (standard heading [mdeg])
    heading.data =  msg->heading;
    // Transform to ENU
    heading_deg.data = (float)heading.data/1000;
    heading_deg.data = 360 - heading_deg.data;    // Orientation increasing Counter clockwise
    heading_deg.data = heading_deg.data + 90;    // 0° pointing East
    if (heading_deg.data > 360.0)
        heading_deg.data = heading_deg.data - 360.0;   
    heading_radians.data = heading_deg.data * M_PI/180;

    // Debug heading readings
    ROS_DEBUG("Heading : [%d] deg ENU: [%f] rad ENU: [%f]", heading.data, heading_deg.data, heading_radians.data);

    //Create imu msg from heading RTK
    heading_rtk.header.stamp = ros::Time::now();
    heading_rtk.header.frame_id = "gps_attitude";
    heading_rtk.orientation = tf::createQuaternionMsgFromYaw(heading_radians.data);  // Create this quaternion from yaw (in radians)
    // Calculated standard deviation for the sensor
    double standard_deviation = sensor_yaw_deviation  * M_PI/180;
    heading_rtk.orientation_covariance[8] = standard_deviation*standard_deviation;   //Update sensor covariance about z axe
    heading_rtk.linear_acceleration.x = 0;
    heading_rtk.linear_acceleration.y = 0;
    heading_rtk.linear_acceleration.z = 0;
    heading_rtk.linear_acceleration_covariance[0] = -1;
    heading_rtk.angular_velocity.x = 0;
    heading_rtk.angular_velocity.y = 0;
    heading_rtk.angular_velocity.z = 0;
    heading_rtk.angular_velocity_covariance[0] = -1;
}

/**
* @brief Node to suscribe and publish heading results
*
******************************************************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "heading_listener");
    ros::NodeHandle n;

    // Susbribers
    ros::Subscriber sub_heading = n.subscribe("/piksi_attitude/baseline_heading", 1000, heading_callback);
    // Publishers
    ros::Publisher pub_heading = n.advertise<sensor_msgs::Imu>("/heading", 1);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        pub_heading.publish(heading_rtk);
        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}

