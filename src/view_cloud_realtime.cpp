/**
* view_cloud_realtime.cpp: utility to view point cloud data in realtime
* Author: Ravi Joshi
* Date: 2018/02/20
*/

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

std::string pointCloudTopic = "/kinect_anywhere/point_cloud/points2";

pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

inline void PointCloudXYZRGBAtoXYZRGB(pcl::PointCloud<pcl::PointXYZRGBA>& in, pcl::PointCloud<pcl::PointXYZRGB>& out)
{
  out.width   = in.width;
  out.height  = in.height;
  out.points.resize(in.points.size());
  for (size_t i = 0; i < in.points.size (); i++)
  {
    out.points[i].x = in.points[i].x;
    out.points[i].y = in.points[i].y;
    out.points[i].z = in.points[i].z;
    out.points[i].r = in.points[i].r;
    out.points[i].g = in.points[i].g;
    out.points[i].b = in.points[i].b;
  }
}

void chatterCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointCloud<pcl::PointXYZRGBA> temp_cloud;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, temp_cloud);
  PointCloudXYZRGBAtoXYZRGB(temp_cloud, cloud);

  if (!viewer.>updatePointCloud(cloud, "cloud"))
      viewer.addPointCloud(cloud, "cloud");

  viewer.spinOnce();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "view_cloud_realtime_node", ros::init_options::AnonymousName);

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(pointCloudTopic, 1, chatterCallback);

  viewer.initCameraParameters();
  viewer.getCameraParameters(argc, argv);

  ros::spin();
  return 0;
}
