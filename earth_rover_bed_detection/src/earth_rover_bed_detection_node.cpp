/*************************************************************
 Copyright (C) 2019 Earth Rover Limited. All rights reserved.
*************************************************************/

/**
 * @file bed_detector_node.cpp
 * @brief Run the bed detector node, which returns whether a tractor or rover is going straight (true) or turning (false).
 * @author Rodrigo Gonzalez (rodrigo.gonzalez@earhtrover.farm)
 * @date May 2019
 */

#include "earth_rover_bed_detection/earth_rover_bed_detection.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "earth_rover_bed_detection");
  ros::NodeHandle nh("earth_rover_bed_detection");
  ros::NodeHandle nh_private("~");
  BedDetection node(nh, nh_private);
  if( node.ok() )
  {
    ros::spin();
  }

  return 0;
}
