#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <sstream>
#include "control_msgs/JointControllerState.h"
#include <cmath>

#define CAM_WIDTH 640
#define CAM_HEIGHT 480
#define PI 3.14
#define RX_LIM PI / 3.6
#define RY_LIM PI / 3.6
#define REFLECT_AXIS_X 1
#define REFLECT_AXIS_Y 1
#define E_SS_LIM 0.01

ros::Publisher panRefPub;
ros::Publisher tiltRefPub;
float current_rx = 0, current_ry = 0;
bool is_pan_moving = false, is_tilt_moving = false;

float calcRef(float target, float *current_r, int maxRes, float r_lim, bool is_reflected){
    float center = maxRes / 2;
    float diff = target - center;
    float new_r = (diff + center) / (maxRes) * r_lim - r_lim / 2;
    if (is_reflected){
        new_r = -new_r;
    }
    new_r += *current_r;
    *current_r = new_r;
    return new_r;
}

void targetXCb(const std_msgs::Float64::ConstPtr& targetX){
    if (is_pan_moving){
        ROS_INFO("Pan is busy");
        return;
    }
    if (targetX->data == 0){
        current_rx = 0;
    }
    else {
        current_rx = calcRef(targetX->data, &current_rx, CAM_WIDTH, RX_LIM, REFLECT_AXIS_X);
    }

    std_msgs::Float64 panRef;
    panRef.data = current_rx;
    panRefPub.publish(panRef);
}

void targetYCb(const std_msgs::Float64::ConstPtr& targetY){
    if (is_tilt_moving){
        ROS_INFO("Tilt is busy");
        return;
    }
    if (targetY->data == 0){
        current_ry = 0;
    }
    else {
        current_ry = calcRef(targetY->data, &current_ry, CAM_HEIGHT, RY_LIM, REFLECT_AXIS_Y);
    }
    std_msgs::Float64 tiltRef;
    tiltRef.data = current_ry;
    tiltRefPub.publish(tiltRef);
}

bool detect_movement(float r, float y, const std::string& name){
    float err = r - y; 
    if (std::abs(err) > E_SS_LIM){
        ROS_INFO("%s is moving. Err: %f", name.c_str(), err);
        return true;
    }
    return false;
}

void pan_state_cb(const control_msgs::JointControllerState::ConstPtr& panState){
    is_pan_moving = detect_movement(current_rx, panState->process_value, "pan");
}

void tilt_state_cb(const control_msgs::JointControllerState::ConstPtr& tiltState){
    is_tilt_moving = detect_movement(current_ry, tiltState->process_value, "tilt");
}


int main(int argc, char **argv){

    ros::init(argc, argv, "pt_cmd");

    ros::NodeHandle nh;
    
    panRefPub = nh.advertise<std_msgs::Float64>("/pan_controller/command", 1);
    tiltRefPub = nh.advertise<std_msgs::Float64>("/tilt_controller/command", 1);

    ros::Subscriber targetXSub = nh.subscribe("targetX", 1, targetXCb);
    ros::Subscriber targetYSub = nh.subscribe("targetY", 1, targetYCb);

    ros::Subscriber panStateSub = nh.subscribe("/pan_controller/state", 1, pan_state_cb);
    ros::Subscriber tiltStateSub = nh.subscribe("/pan_controller/state", 1, tilt_state_cb);

    while (true){
        ros::spinOnce();
    }
    
}