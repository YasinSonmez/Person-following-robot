// PCL specific includes
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <string>
#include <stdexcept>
 #include <ros/ros.h>
#include <ros/callback_queue.h>
#include <pcl/conversions.h>
#include <pcl/common/time.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
//#include  <chrono
#include <vector>
#include <ctime>
#include <pcl/common/time.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>

#include <opencv2/core.hpp>     // Basic OpenCV structures (cv::Mat)
#include <opencv2/videoio.hpp>  // Video write
#include <pcl/io/pcd_io.h>
#include <cstdio>
#include <ctime>
#include <pcl_conversions/pcl_conversions.h>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono> 
#include <helpers.h> 

#define HEIGHT 480
#define WIDTH 640


pcl::PointCloud<pcl::PointXYZRGB> pc_xyzrgb;



void pc_cb(const sensor_msgs::PointCloud2 &pc_msg){
  pcl::PCLPointCloud2 temp;
  pcl_conversions::toPCL(pc_msg, temp);
  pcl::fromPCLPointCloud2(temp, pc_xyzrgb);
  for (auto& point: pc_xyzrgb){
    point.z = -point.z;
    point.y = -point.y;
    point.x = -point.x;
  }

  pcl::io::savePCDFileASCII("/home/malici/navi_ws/src/localize-and-navigate/obj_detection/src/pcd_data/test_pcd.pcd", pc_xyzrgb);
  std::cerr << "Saved " << pc_xyzrgb.size () << " data points to test_pcd.pcd." << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_segmenter");
  ros::NodeHandle n;
  ros::Subscriber sub_depth = n.subscribe("/zed/depth/points", 1000, pc_cb);

  while (ros::ok()){
      ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
      //visualize_pc(pc_xyzrgb);
  }

  ros::spin();

  return 0;
}
