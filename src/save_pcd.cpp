#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

std::string pointCloudTopic = "/kinect_anywhere/point_cloud/points2";
std::string saveDir = "/home/baxterpc/ros_ws/src/multiple_kinect_baxter_calibration/files/captured_pcd";

int file_index = 0;
void chatterCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg)
{
  ROS_INFO("Point Cloud Received.");

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromPCLPointCloud2(pcl_pc2, cloud);

  std::stringstream filename;
  filename << "/file_" << (file_index++) << ".pcd";
  std::string full_path = saveDir + filename.str();
  pcl::io::savePCDFileASCII (full_path, cloud);
  ROS_INFO_STREAM("Point Cloud Converted. Points: " << (cloud.width * cloud.height));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_pcd", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(pointCloudTopic, 1, chatterCallback);
  ros::spin();
  return 0;
}
