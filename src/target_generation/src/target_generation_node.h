#include "ros/ros.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h> // tf to geometry_msgs conversion
#include "ros/topic.h"
#include <std_msgs/String.h>
#include <math.h> // atan
#include <leg_tracker/PersonArray.h>

using namespace std;

#define PI 3.14159265
// Distance that is substracted from the position of the actor
#define DISTANCE_TO_FOLLOW_BEHIND 2.0
// #define FOLLOW_MODE "check costmap behind the human"
#define FOLLOW_MODE "between the robot and human"
// #define FOLLOW_MODE "behind the human"
// #define HUMAN_TRACKING_MODE "laser"
// #define HUMAN_TRACKING_MODE "gazebo"
#define HUMAN_TRACKING_MODE "camera"

class TargetNode
{
private:
  // Callbacks
  void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &model_states);
  void modelStatesForDistanceCallback(const gazebo_msgs::ModelStates::ConstPtr &model_states);
  void peopleTrackedCallback(const leg_tracker::PersonArray::ConstPtr &person_array); // leg
  void humanCallback(const geometry_msgs::PoseStamped::ConstPtr &human_pos);          // camera
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &costmap_msg);

  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber modelStates_sub;
  ros::Subscriber modelStatesForDistance_sub;
  ros::Subscriber peopleTracked_sub;
  ros::Subscriber human_sub;
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
  bool actor_message_arrived = false;
  double yaw = 0.0; // yaw angle of the followed human

  // Variables for path distance calculation
  double total_distance_robot = 0, total_distance_human = 0, total_distance = 0,
         average_distance = 0, previous_x_robot = 0, previous_y_robot = 0,
         previous_x_human = 0, previous_y_human = 0;
  int avg_count = 1;
  double first_time = 0;
  geometry_msgs::PoseStamped actor_pose_ground_truth;

  ros::Time begin;

public:
  TargetNode();
  void mainLoop();
};