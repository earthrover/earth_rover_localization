#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::TransformStamped tf_ugv_od_ugv_bl;
//tf::StampedTransform *tf_scout_bl_ugv_bl;
ros::Publisher *pub;  


void cb_local_tf(const geometry_msg::PoseStamped& msg){
  geometry_msgs::PoseStamped pose_ugv_od_to_ugv_bl;
  tf2::Transform ugv_od_ugv_bl_transform;
  
  tf2::fromMsg(tf_ugv_od_ugv_bl.transform, ugv_od_ugv_bl_transform); // Get transformation matrix from tf
  tf2::toMsg(ugv_od_ugv_bl_transform, ugv_od_to_ugv_bl); // Create pose msg from transformation matrix

  pub.publish(ugv_od_to_ugv_bl);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "local_transform_node");

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("odom/filtered/local", 50, &cb_local_tf);
    pub = new node.advertise<geometry_msgs::PoseStamped>("/odom/filtered/ugv_nav", 100);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


    ros::Rate rate(20.0);
    while (node.ok()){
        try{
        tf_ugv_od_ugv_bl = tfBuffer.lookupTransform("/ugv_odom", "/ugv_base_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex){
          ROS_ERROR("%s",ex.what());
        }
    }



  return 0;
};