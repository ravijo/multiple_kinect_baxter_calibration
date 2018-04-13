/**
 * view_cloud.cpp: utility to view point cloud from PCD file
 * Author: Ravi Joshi
 * Date: 2018/02/20
 */

// ros headers
#include <ros/ros.h>

// pcl headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

void pointPickingEventOccurred(
  const pcl::visualization::PointPickingEvent& event, void* viewer_void) {
  if (event.getPointIndex() == -1)
    return;

  float x, y, z;
  event.getPoint(x, y, z);
  ROS_INFO_STREAM(
  "Point picking event occurred. Point coordinate ( " << x << ", "
  << y << ", " << z << ")");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "view_cloud_node", ros::init_options::AnonymousName);

  ros::NodeHandle nh("~");
  std::string pcd_file;
  nh.getParam("file", pcd_file);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
  new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile(pcd_file, *cloud);

  // load PCD file
  if (pcl::io::loadPCDFile < pcl::PointXYZRGB > (pcd_file, *cloud) == -1) {
    ROS_ERROR_STREAM("Couldn't read file input file " << pcd_file);
    return -1;
  }

  ROS_INFO_STREAM("Hold down 'SHIFT' key while left-clicking to pick a point.");

  pcl::visualization::PCLVisualizer viewer(
  "Cloud Viewer (Press 's' to save screenshot)");
  viewer.addPointCloud(cloud, "cloud");

  viewer.initCameraParameters();
  viewer.getCameraParameters(argc, argv);
  viewer.registerPointPickingCallback(pointPickingEventOccurred,
  (void*) &viewer);

  viewer.spin();

  ros::spin();
  return 0;
}
