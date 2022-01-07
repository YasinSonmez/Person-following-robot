#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void visualize_pc(PointCloudT::Ptr pc){
  pcl::visualization::CloudViewer viewer("Simple cloud viewer");
  viewer.showCloud(pc);
  while (!viewer.wasStopped());
}


int main(int argc, char **argv){
    std::string path = argv[1];
    PointCloudT::Ptr cloud (new PointCloudT);
    if (pcl::io::loadPCDFile<PointT> (path, *cloud) == -1){
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
                << cloud->width * cloud->height
                << " data points from test_pcd.pcd"
                << std::endl;

    PointCloudT::Ptr cloud_oriented(new PointCloudT);


    visualize_pc(cloud);

    return 0;


}