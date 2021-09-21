#include "ros/ros.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
using namespace std;

#define PI 3.14159265

class TargetNode
{
private:
  // Callbacks
  void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &model_states);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &costmap_msg);

  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber modelStates_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber costmap_sub;

  // Publishers
  ros::Publisher target_pub;
  ros::Publisher followed_person_pub;

  // Variables
  geometry_msgs::PoseStamped target;
  geometry_msgs::PoseStamped actor_pose;
  nav_msgs::Odometry odom;
  nav_msgs::OccupancyGrid costmap;
  geometry_msgs::PoseStamped odom_in_map_frame;
  geometry_msgs::TransformStamped odom_to_map;
  tf2_ros::Buffer tf_buffer;
  //string follow_mode = "between the robot and human";
  string follow_mode = "behind the human";
  //string follow_mode = "check costmap behind the human";

  ros::Time begin;

public:
  TargetNode();
  void mainLoop();
};