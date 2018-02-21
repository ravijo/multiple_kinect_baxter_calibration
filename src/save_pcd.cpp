/**
* save_pcd.cpp: utility to save point cloud data to file
* Author: Ravi Joshi
* Date: 2018/02/20
*/

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


int file_index = 0;
std::string pointCloudTopic = "/kinect1/sd/points";
std::string saveDir = "/home/baxterpc/ros_ws/src/multiple_kinect_baxter_calibration/files/captured_pcd";

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
  ROS_INFO("Point Cloud Received.");

  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_conversions::toPCL(*msg, pcl_pc2);

  if(msg->fields[3].name == "rgb")
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud); // for libfreenect
  else
  {
    // for kinect_anywhere
    pcl::PointCloud<pcl::PointXYZRGBA> temp_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, temp_cloud);
    PointCloudXYZRGBAtoXYZRGB(temp_cloud, *cloud);
  }

  std::stringstream filename;
  filename << "/file_" << (file_index++) << ".pcd";
  std::string full_path = saveDir + filename.str();
  pcl::io::savePCDFileASCII (full_path, *cloud);
  ROS_INFO_STREAM("Point Cloud Converted. Points: " << (cloud->width * cloud->height));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_pcd", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(pointCloudTopic, 1, chatterCallback);
  ros::spin();
  return 0;
}
