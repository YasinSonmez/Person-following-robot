#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "geometry_msgs/Point.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <sensor_msgs/image_encodings.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <iostream>
#include <sys/time.h>
#include <vector>

#include "kalman_filter.h"
#include <geometry_msgs/PoseStamped.h>
#include <numeric>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#define PI 3.14159265

using namespace std;

double targetX, targetY, prev_targetX, prev_targetY, width, height;
double targetX_c, targetY_c, width_c, height_c;
const int depthStepSize = 2;
const double kinectFovX = 30.0;
double targetPositionY = 0.0, targetPositionX = 0.0;
double prev_targetPositionX = 0, prev_targetPositionY = 0;
double yaw = 0;

static bool detected_human = false;
static bool prev_found = false;
static bool initialized = false;

int prev_blobArea, blobArea;
const double distBtwBlobCenters_limit = 120;
const double blobAreaChangeLimit = 0.2;
double distBtwBlobCenters;
long int min_vicinity;

int fail_count = 0;
const int max_fail_count = 10;

class YoloPostProcessing
{
private:
  ros::NodeHandle nh_;
  ros::Publisher target_position_pub;
  ros::Publisher targetX_pub;
  KalmanFilter kalman_filter_;
  tf::TransformListener tf_listener_;
  tf::StampedTransform zed_camera_center_transform_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb_sub;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

public:
  YoloPostProcessing()
  {
    depth_sub.subscribe(nh_, "/zed/depth/image_raw", 1);
    bb_sub.subscribe(nh_, "/darknet_ros/bounding_boxes", 1);
    sync_.reset(new Sync(MySyncPolicy(10), depth_sub, bb_sub));
    sync_->registerCallback(boost::bind(&YoloPostProcessing::callback, this, _1, _2));
    target_position_pub =
        nh_.advertise<geometry_msgs::PoseStamped>("human_pose", 1);
    targetX_pub =
        nh_.advertise<std_msgs::Int16>("targetX", 1);
    wait_transform();
  }

  void callback(const sensor_msgs::ImageConstPtr &depth_msg, const darknet_ros_msgs::BoundingBoxes::ConstPtr &bb_msg)
  {
    // Bounding Box calculation
    if (!initialized)
    {
      initialized = true;

      targetX = 0;
      targetY = 0;
      width = 0;
      height = 0;
      prev_blobArea = 0;
      fail_count = 0;
      min_vicinity = 400000; // starts with very high number
    }

    for (int i = 0; i < bb_msg->bounding_boxes.size(); i++)
    {
      if (bb_msg->bounding_boxes[i].Class == "person")
      {

        width_c = bb_msg->bounding_boxes[i].xmax - bb_msg->bounding_boxes[i].xmin;
        targetX_c = bb_msg->bounding_boxes[i].xmin + (width_c) / 2;
        height_c = bb_msg->bounding_boxes[i].ymax - bb_msg->bounding_boxes[i].ymin;
        targetY_c = bb_msg->bounding_boxes[i].ymin + (height_c) / 2;

        blobArea = width_c * height_c;

        // ROS_INFO("prev_blob_area : %d blob_area : %d", prev_blobArea, blobArea);

        if (prev_blobArea != 0)
        {
          distBtwBlobCenters = sqrt(pow(targetX_c - prev_targetX, 2) +
                                    pow(targetY_c - prev_targetY, 2));
        }
        else
        {
          prev_blobArea = blobArea;
          distBtwBlobCenters = distBtwBlobCenters_limit - 1;
        }

        if (distBtwBlobCenters <
            distBtwBlobCenters_limit) // && abs(blobArea- prev_blobArea) < (long
                                      // int) (prev_blobArea *
                                      // blobAreaChangeLimit))
        {
          long int vicinity = (int)distBtwBlobCenters;
          if (vicinity < min_vicinity)
          {
            min_vicinity = vicinity;
            width = width_c;
            targetX = targetX_c;
            height = height_c;
            targetY = targetY_c;
            detected_human = true;
            std_msgs::Int16 targetX_msg;
            targetX_msg.data = (int)targetX;
            targetX_pub.publish(targetX_msg);
          }
        }
      }
    }

    // if not detected human for some time, reset searching to initial values
    if (!detected_human)
    {
      if (fail_count < 10)
      {
        fail_count++;
      }
      else
      {
        fail_count = 0;
        width = 0;
        height = 0;
        initialized = false;
      }
    }
    else
    {
      fail_count = 0;
    }

    // getting ready for the new loop
    prev_targetX = targetX;
    prev_targetY = targetY;
    prev_blobArea = width * height;
    min_vicinity = 400000; // very high number

    // Depth calculation
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(depth_msg);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    if (detected_human)
    {
      detected_human = false;
      vector<float> dists;

      for (int i = -width / 4; i <= width / 4; i += depthStepSize)
      {
        for (int j = -height / 4; j <= height / 4;
             j += depthStepSize)
        {
          auto dist_ij =
              cv_ptr->image.at<float>(cv::Point(targetX + i, targetY + j));
          if (!isnan(dist_ij))
          {
            dists.push_back(dist_ij);
            // targetDistance = dist_ij;
          }
        }
      }

      // Remove the points far away than average 3 times
      for (int i = 0; i < 3; i++)
      {
        if (!dists.empty())
        {
          float avg = accumulate(dists.begin(), dists.end(), 0.0) / dists.size();
          auto it = remove_if(dists.begin(), dists.end(), bind2nd(greater<float>(), avg + 1));
          dists.erase(it, dists.end());
        }
      }

      if (!dists.empty())
      {
        float targetDistance = accumulate(dists.begin(), dists.end(), 0.0) / dists.size();
        // cout << "before: " << avg << "       after: " << targetDistance << endl;
        targetPositionY =
            -targetDistance *
            sin((kinectFovX * ((targetX - 320) / 320.0)) * (PI / 180));
        targetPositionX =
            sqrt(pow(targetDistance, 2) - pow(targetPositionY, 2));

        Eigen::Matrix<double, kNumStates, 1> measurement;
        cout << "Before: " << targetPositionX << ",   " << targetPositionY << endl;
        TransformPose(targetPositionX, targetPositionY);
        cout << "After: " << targetPositionX << ",   " << targetPositionY << endl;

        // If the difference is more than threshold, update angle
        double y_diff = targetPositionY - prev_targetPositionY;
        double x_diff = targetPositionX - prev_targetPositionX;
        double total_distance = abs(y_diff) + abs(x_diff);
        cout << "total distance: " << total_distance << endl;
        if (total_distance > 0.4)
        {
          yaw = std::atan2(y_diff, x_diff);
          prev_targetPositionX = targetPositionX;
          prev_targetPositionY = targetPositionY;
        }

        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header.frame_id = "map";
        poseMsg.header.stamp = ros::Time::now();
        poseMsg.pose.position.x = targetPositionX;
        poseMsg.pose.position.y = targetPositionY;
        poseMsg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        target_position_pub.publish(poseMsg);
        prev_found = true;
      }
    }
    else
    {
      // If no bounding box is present, previous results are published
      prev_found = false;
      geometry_msgs::PoseStamped poseMsg;
      poseMsg.header.frame_id = "map";
      poseMsg.header.stamp = ros::Time::now();
      poseMsg.pose.position.x = targetPositionX;
      poseMsg.pose.position.y = targetPositionY;
      poseMsg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

      target_position_pub.publish(poseMsg);
    }
  }
  void mainLoop()
  {
    if (!prev_found)
    {
      // If no bounding box is present, previous results are published
      geometry_msgs::PoseStamped poseMsg;
      poseMsg.header.frame_id = "map";
      poseMsg.header.stamp = ros::Time::now();
      poseMsg.pose.position.x = targetPositionX;
      poseMsg.pose.position.y = targetPositionY;
      poseMsg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

      target_position_pub.publish(poseMsg);
    }
    prev_found = false;
  }

  void wait_transform()
  {
    while (true)
    {
      try
      {
        tf_listener_.waitForTransform("map", "zed_camera_center", ros::Time::now(), ros::Duration(1.0));
        tf_listener_.lookupTransform("map", "zed_camera_center", ros::Time(0), zed_camera_center_transform_);
        break;
      }
      catch (tf::TransformException ex)
      {
        ROS_WARN_STREAM("Waiting for transform between "
                        << "zed_camera_center"
                        << " - "
                        << "map"
                        << " . It is not initialized yet!");
      }
    }
  }

  void TransformPose(double &x, double &y)
  {
    tf_listener_.lookupTransform("map", "zed_camera_center", ros::Time(0), zed_camera_center_transform_);
    tf::Vector3 current_position(x, y, 0.0);
    current_position = zed_camera_center_transform_(current_position);
    x = current_position.getX();
    y = current_position.getY();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "yolo_pp");
  YoloPostProcessing yolo_pp;
  cout << "Post processing module started!" << endl;
  ros::Rate rate(10);

  while (ros::ok())
  {
    yolo_pp.mainLoop();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}