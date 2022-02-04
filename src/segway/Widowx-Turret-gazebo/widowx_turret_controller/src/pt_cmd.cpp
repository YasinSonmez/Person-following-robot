#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <sstream>
#include "control_msgs/JointControllerState.h"
#include <cmath>
#include <tf/tf.h>
using namespace std;

#define CAM_WIDTH 640
#define CAM_HEIGHT 480
#define PI 3.14
#define RX_LIM PI / 3.6
#define RY_LIM PI / 3.6
#define REFLECT_AXIS_X 1
#define REFLECT_AXIS_Y 1
#define E_SS_LIM 0.15

// Publisher
ros::Publisher panRefPub;
// Variables
float prev_rx = 0, current_rx = 0, angle_added = 0,
      prev_angle_added = 0, yaw = 0, angle_increment = 0.06;
nav_msgs::Odometry odom;

// Counters
int is_human_detected = 0, dont_increase_count = 0, lost_count = 0;

float mod(float a, float N) { return a - N * floor(a / N); } // Mod function to return in range [0, N)

void targetXCb(const std_msgs::Int16::ConstPtr &targetX)
{
    // When human is detected start counter from 200
    is_human_detected = 200;
}

void panStateCb(const control_msgs::JointControllerState::ConstPtr &panState)
{
    // Decrease the count, if human isn't seen for a while count becomes 0
    if (is_human_detected)
        is_human_detected--;
}

void odomCb(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    odom = *odom_msg;

    // Get the yaw angle from quaternion
    tf::Pose pose;
    tf::poseMsgToTF(odom.pose.pose, pose);
    yaw = -tf::getYaw(pose.getRotation());
}

void humanPoseCb(const geometry_msgs::PoseStamped::ConstPtr &human_pose)
{
    // calculate the angle to focus on human
    double focus_angle = atan2((human_pose->pose.position.y - odom.pose.pose.position.y),
                               (human_pose->pose.position.x - odom.pose.pose.position.x));
    std_msgs::Float64 panRef;
    if (is_human_detected)
    {
        // add yaw to focus_angle to account for the body's motion
        current_rx = mod((yaw + focus_angle + PI), (2 * PI)) - PI;

        panRef.data = current_rx;
        panRefPub.publish(panRef);

        angle_added = 0;
        prev_rx = current_rx;
    }
    else
    {
        // If turning 2*pi to catch up, don't increase angle for some time
        if (abs(prev_rx - current_rx) > 0.1)
            dont_increase_count = 40;
        // Increase the angle with small increments to find the human
        if (!dont_increase_count)
            angle_added = mod((angle_added + angle_increment + PI), (2 * PI)) - PI;

        prev_rx = current_rx;
        current_rx = mod((yaw + focus_angle + angle_added + PI), (2 * PI)) - PI;

        // If the head moves 2*pi degrees count as lost
        if (prev_angle_added < 0 && angle_added > 0)
        {
            lost_count++;
            cout << " Lost count: " << lost_count << endl;
        }
        prev_angle_added = angle_added;
        panRef.data = current_rx;
        panRefPub.publish(panRef);

        if (dont_increase_count)
            dont_increase_count--;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pt_cmd");

    ros::NodeHandle nh;

    panRefPub = nh.advertise<std_msgs::Float64>("/pan_controller/command", 1);

    ros::Subscriber targetXSub = nh.subscribe("targetX", 1, targetXCb);
    ros::Subscriber odomSub = nh.subscribe("base_pose_ground_truth", 1, odomCb);
    ros::Subscriber panStateSub = nh.subscribe("/pan_controller/state", 1, panStateCb);
    ros::Subscriber humanPoseSub = nh.subscribe("human_pose", 1, humanPoseCb);

    while (true)
    {
        ros::spinOnce();
    }
}