#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <mutex>
#include <thread>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <iostream>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

// Mutex: //
std::mutex cloud_mutex;
bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);
PointCloudT pc;

enum { COLS = 640, ROWS = 480 };


void pc_cb_2(const sensor_msgs::PointCloud2 &pc_msg){
    std::cout << "in cb\n";
    pcl::PCLPointCloud2 temp;
    pcl_conversions::toPCL(pc_msg, temp);
    pcl::fromPCLPointCloud2(temp, pc);
    std::cout << "after conv\n";
    for (auto& point: pc){
    point.z = -point.z;
    point.y = -point.y;
    point.x = -point.x;
    point.rgb = *reinterpret_cast<float*>(&point.rgb);
    }
    cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
    new_cloud_available_flag = true;
    *cloud = pc;
    cloud_mutex.unlock ();
    std::cout << "out cb\n";
    
}

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};
  
void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

int main (int argc, char** argv)
{
  // Algorithm parameters:
  std::string svm_filename = "/home/malici/navi_ws/src/localize-and-navigate/obj_detection/src/people/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  float min_confidence = -1.5;
  float min_height = 1.3;
  float max_height = 2.3;
  float voxel_size = 0.06;
  Eigen::Matrix3f rgb_intrinsics_matrix;
  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

  // Read if some parameters are passed from command line:
  pcl::console::parse_argument (argc, argv, "--svm", svm_filename);
  pcl::console::parse_argument (argc, argv, "--conf", min_confidence);
  pcl::console::parse_argument (argc, argv, "--min_h", min_height);
  pcl::console::parse_argument (argc, argv, "--max_h", max_height);
  std::cout << "before cb\n";
  // Init ROS
  ros::init(argc, argv, "pc_segmenter");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/zed/depth/points", 1000, pc_cb_2);
  std::cout << "after cb\n";
  //ros::spin();

  while(ros::ok()) {
    std::cout << "in while\n";
    while (!new_cloud_available_flag){
      ros::spinOnce();
    }    
    std::cout << "after spin\n";
    cloud_mutex.lock();    // for not overwriting the point cloud

    // Display pointcloud:
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer.addPointCloud<PointT> (cloud, rgb, "sample cloud");
    viewer.setBackgroundColor (0, 0, 0);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer.addCoordinateSystem (1.0);
    viewer.initCameraParameters ();
    // pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    // viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
    // viewer.setCameraPosition(0,0,-2,0,-1,0,0);

    // Add point picking callback to viewer:
    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d (new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
    viewer.registerPointPickingCallback (pp_callback, (void*)&cb_args);
    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

    // Spin until 'Q' is pressed:
    viewer.spin();
    std::cout << "done." << std::endl;
    
    cloud_mutex.unlock ();

    // Ground plane estimation:
    Eigen::VectorXf ground_coeffs;
    ground_coeffs.resize(4);
    std::vector<int> clicked_points_indices;
    for (unsigned int i = 0; i < clicked_points_3d->size(); i++)
      clicked_points_indices.push_back(i);
    pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
    model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
    std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;

    // Initialize new viewer:
    // pcl::visualization::PCLVisualizer viewer("PCL Viewer");          // viewer initialization
    // viewer.setCameraPosition(0,0,-2,0,-1,0,0);

    // Create classifier for people detection:  
    pcl::people::PersonClassifier<pcl::RGB> person_classifier;
    person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

    // People detection app initialization:
    pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
    people_detector.setVoxelSize(voxel_size);                        // set the voxel size
    people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
    people_detector.setClassifier(person_classifier);                // set person classifier
    people_detector.setPersonClusterLimits(min_height, max_height, 0.1, 8.0);  // set person classifier
  //  people_detector.setSensorPortraitOrientation(true);             // set sensor orientation to vertical

    // For timing:
    static unsigned count = 0;
    static double last = pcl::getTime ();

    // Main loop:
    while (!viewer.wasStopped() && ros::ok())
    {
      ros::spinOnce();
      if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
      {
        new_cloud_available_flag = false;

        // Perform people detection on the new cloud:
        std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
        people_detector.setInputCloud(cloud);
        people_detector.setGround(ground_coeffs);                    // set floor coefficients
        people_detector.compute(clusters);                           // perform people detection

        ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

        // Draw cloud and people bounding boxes in the viewer:
        viewer.removeAllPointClouds();
        viewer.removeAllShapes();
        // pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
        // viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");

        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
        viewer.addPointCloud<PointT> (cloud, rgb, "sample cloud");


        unsigned int k = 0;
        for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
        {
          if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
          {
            // draw theoretical person bounding box in the PCL viewer:
            it->drawTBoundingBox(viewer, k);
            k++;
          }
        }
        std::cout << k << " people found" << std::endl;
        viewer.spinOnce();

        // Display average framerate:
        if (++count == 30)
        {
          double now = pcl::getTime ();
          std::cout << "Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
          count = 0;
          last = now;
        }
        cloud_mutex.unlock ();
      }
    }
  }
 
}