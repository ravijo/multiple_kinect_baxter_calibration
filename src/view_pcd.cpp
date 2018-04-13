/**
 * view_pcd.cpp: utility to view point cloud from PCD file
 * Author: Ravi Joshi
 * Date: 2018/02/20
 */

// ros headers
#include <ros/ros.h>

// pcl headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

/*
void keyboardEventOccurred(
  const pcl::visualization::KeyboardEvent& event, void* viewer) {
    if (event.getKeySym() == "q" && event.keyDown()){
        ROS_INFO_STREAM("Key 'q' was pressed. Exiting...");
        pcl::visualization::PCLVisualizer* v
            = static_cast<pcl::visualization::PCLVisualizer*>(viewer);
        v->close(); // doesn't work with PCL 1.7
    }
}
*/

void pointPickingEventOccurred(
  const pcl::visualization::PointPickingEvent& event, void* viewer) {
  if (event.getPointIndex() == -1)
    return;

  float x, y, z;
  event.getPoint(x, y, z);
  ROS_INFO_STREAM(
  "Point picking event occurred. Point coordinate ( " << x << ", "
  << y << ", " << z << ")");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "view_pcd_node", ros::init_options::AnonymousName);

  ros::NodeHandle nh("~");
  std::string pcd_file;
  nh.getParam("file", pcd_file);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
  new pcl::PointCloud<pcl::PointXYZRGB>);

  // load PCD file and check if invalid file is provided
  if (pcl::io::loadPCDFile < pcl::PointXYZRGB > (pcd_file, *cloud) == -1) {
    ROS_ERROR_STREAM("Couldn't read input file " << pcd_file);
    return -1;
  }else{
    ROS_INFO_STREAM("Loading '" << pcd_file << "'");
  }

  ROS_INFO_STREAM("Hold down 'SHIFT' key while left-clicking to pick a point.");

  pcl::visualization::PCLVisualizer viewer(
  "Cloud Viewer (Press 'j' to take screenshot)");

  viewer.addPointCloud(cloud, "cloud");

  viewer.initCameraParameters();
  viewer.getCameraParameters(argc, argv);
  viewer.registerPointPickingCallback(pointPickingEventOccurred,
  (void*) &viewer);
  //viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

  viewer.spin();

  ros::spin();
  return 0;
}
