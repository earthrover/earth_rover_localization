/**
 * @file remove_gps_latency.cpp
 * @brief This file substracts the GPS measured latency to the GPS message
 * @author Xavier Ruiz (xavier.ruiz@earthrover.farm)
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <string>


class RemoveGpsLatency
 {
public:
     RemoveGpsLatency(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle)
     : nh_(node_handle), pnh_(private_node_handle)
     {
         this->init();
     }
     ~RemoveGpsLatency() = default;

     void init();
     void gpsCallback(const sensor_msgs::NavSatFixConstPtr& msg);
     void headingCallback(const sensor_msgs::ImuConstPtr& heading_msg);
     // Initialize Variables
     double gps_latency_{0.0};
     // public and private ros node handle
     ros::NodeHandle nh_;
     ros::NodeHandle pnh_;
     // Subscribers and publishers
     ros::Subscriber gps_sub_;
     ros::Subscriber heading_sub_;
     ros::Publisher gps_pub_;
     ros::Publisher heading_pub_;
};

void RemoveGpsLatency::init()
{
      // Load params
      nh_.getParam("/remove_gps_latency/gps_latency", gps_latency_);

      gps_sub_ = nh_.subscribe("/piksi_receiver/navsatfix_best_fix", 1000, &RemoveGpsLatency::gpsCallback, this);
      heading_sub_ = nh_.subscribe("/heading", 1000, &RemoveGpsLatency::headingCallback, this);
      gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/piksi_receiver/navsatfix_best_fix/corrected_latency", 1000);
      heading_pub_ = nh_.advertise<sensor_msgs::Imu>("/heading/corrected_latency", 1000);
}


void RemoveGpsLatency::gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg)
{
      sensor_msgs::NavSatFix gps_latency_msg;
      gps_latency_msg = *gps_msg;
      ros::Duration gps_latency_2rosdur(gps_latency_*0.001);
      gps_latency_msg.header.stamp = gps_msg->header.stamp - gps_latency_2rosdur;
      gps_pub_.publish(gps_latency_msg);
}

void RemoveGpsLatency::headingCallback(const sensor_msgs::ImuConstPtr& heading_msg)
{
     sensor_msgs::Imu heading_latency_msg;
     heading_latency_msg = *heading_msg;
     ros::Duration gps_latency_2rosdur(gps_latency_*0.001);
     heading_latency_msg.header.stamp = heading_msg->header.stamp - gps_latency_2rosdur;
     heading_pub_.publish(heading_latency_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "remove_gps_latency");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    RemoveGpsLatency node(nh, nh_private);
    ros::spin();

    return 0;
}
