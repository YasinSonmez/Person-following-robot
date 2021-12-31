#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

bool is_initial {true};
geometry_msgs::Pose init_pose {};
geometry_msgs::TransformStamped base_trans{};
geometry_msgs::TransformStamped odom_trans{};


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

  if (is_initial){
    init_pose.position = msg->pose.pose.position;
    init_pose.orientation = msg->pose.pose.orientation;

    std::cout << "==========================" << std::endl;
    std::cout << init_pose.orientation.x << std::endl;
    std::cout << init_pose.orientation.y << std::endl;
    std::cout << init_pose.orientation.z << std::endl;
    std::cout << init_pose.orientation.w << std::endl;
    is_initial = false;
  }
  base_trans.header.stamp = ros::Time::now();
  base_trans.header.frame_id = "world";
  base_trans.child_frame_id = "base_link";
  base_trans.transform.translation.x = msg->pose.pose.position.x;
  base_trans.transform.translation.y = msg->pose.pose.position.y;
  base_trans.transform.translation.z = msg->pose.pose.position.z;
  base_trans.transform.rotation = msg->pose.pose.orientation;


  odom_trans = base_trans;
  odom_trans.child_frame_id = "odom";
  odom_trans.transform.translation.x = init_pose.position.x;
  odom_trans.transform.translation.y = init_pose.position.y;
  odom_trans.transform.translation.z = init_pose.position.z;
  odom_trans.transform.rotation = init_pose.orientation;


}

int main(int argc, char **argv) {
  ros::init(argc,argv,"gazebo_tf_pub");
  ros::NodeHandle nh;
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster base_broadcaster;
  ros::Rate rate(100);
  ros::Subscriber sub = nh.subscribe("odom", 1, odomCallback);
  while (ros::ok()){
    if (!is_initial){
      odom_broadcaster.sendTransform(odom_trans);
      base_broadcaster.sendTransform(base_trans);
    }

    rate.sleep();
    ros::spinOnce();
  }



  return 0;
}
