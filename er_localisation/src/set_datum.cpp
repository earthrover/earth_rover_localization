/**
 * @file set_datum.cpp
 * @brief Find convergence value using given location on .yaml file. Set datum param, origin of map to navsat_transform_node on robot_localization
 * @author Andres Palomino (apalomino@edgebrain.io)
 */

#include "ros/ros.h"
#include <geographic_msgs/GeoPose.h>
#include <robot_localization/SetDatum.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <GeographicLib/GeoCoords.hpp>


using namespace GeographicLib;

/**
* @brief Client Node. Takes localization parameters to set orogin of map
*
******************************************************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_datum_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<robot_localization::SetDatum>("datum");
    robot_localization::SetDatum srv;

    double latitude0_param, longitude0_param, altitude0_param, convergence0_param;

    // Get Params of ENU origin
    if (ros::param::has("latitude0_deg") && ros::param::has("longitude0_deg") && ros::param::has("altitude0") )
    {        
        ros::param::get("latitude0_deg", latitude0_param);
        ros::param::get("longitude0_deg", longitude0_param);
        ros::param::get("altitude0", altitude0_param);                     
        ROS_INFO("Using params from .yaml ");
        ROS_INFO("latitude0_deg [%f] longitude0_deg [%f] altitude0 [%f]", latitude0_param, longitude0_param, altitude0_param);
    }

    // Create geo_pose message
    geographic_msgs::GeoPose geo_pose;
    geo_pose.position.latitude = latitude0_param;
    geo_pose.position.longitude = longitude0_param;
    geo_pose.position.altitude = altitude0_param;
    // Find convergence value from location
    GeoCoords conversion_data(latitude0_param, longitude0_param);
    float convergence_radians = conversion_data.Convergence() * M_PI/180.0;
    geo_pose.orientation = tf::createQuaternionMsgFromYaw( convergence_radians );  // Create this quaternion from yaw (in radians)
    ROS_INFO("convergence [%f] ", convergence_radians);

    // Call service
    srv.request.geo_pose = geo_pose;
    if (client.call(srv))
    {        
    }
    else
    {
        ROS_ERROR("Failed to call datum service");
        return 1;
    }

    return 0;
}

