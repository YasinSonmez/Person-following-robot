#include "ros/ros.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
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

  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber odom_sub;
  ros::Subscriber modelStates_sub;

  // Publishers
  ros::Publisher target_pub;
  ros::Publisher followed_person_pub;

  // Variables
  geometry_msgs::PoseStamped target;
  geometry_msgs::PoseStamped actor_pose;
  nav_msgs::Odometry odom;
  geometry_msgs::PoseStamped odom_in_map_frame;
  geometry_msgs::TransformStamped odom_to_map;
  tf2_ros::Buffer tf_buffer;

  ros::Time begin;

public:
  TargetNode();
  void mainLoop();
};