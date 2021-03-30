#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::TransformStamped tf_ugv_od_ugv_bl;
//tf::StampedTransform *tf_scout_bl_ugv_bl;
ros::Publisher pub;  


void cb_local_tf(const geometry_msgs::PoseStamped& msg){
  geometry_msgs::Pose pose_ugv_od_to_ugv_bl;
  geometry_msgs::PoseStamped final_pose_ugv;
  tf2::Transform ugv_od_ugv_bl_transform;
  
  tf2::fromMsg(tf_ugv_od_ugv_bl.transform, ugv_od_ugv_bl_transform); // Get transformation matrix from tf
  tf2::toMsg(ugv_od_ugv_bl_transform, pose_ugv_od_to_ugv_bl); // Create pose msg from transformation matrix
  ROS_INFO("OK");
  final_pose_ugv.header = msg.header;
  final_pose_ugv.header.frame_id = "ugv_odom";
  final_pose_ugv.pose = pose_ugv_od_to_ugv_bl;
  pub.publish(final_pose_ugv);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "local_transform_node");
    ros::Time::init();    
    ros::Rate r(10);
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("odometry/filtered/local", 1000, &cb_local_tf);
    pub = node.advertise<geometry_msgs::PoseStamped>("/odometry/filtered/ugv_nav", 1);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


     while (node.ok()){
        try{
	tf_ugv_od_ugv_bl = tfBuffer.lookupTransform( "ugv_odom", "ugv_base_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex){
          ROS_ERROR("%s",ex.what());
          continue;
        }
        ros::spinOnce();
        r.sleep();
    }



  return 0;
};
