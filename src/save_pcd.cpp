/**
 * save_pcd.cpp: utility to save point cloud data to file
 * Author: Ravi Joshi
 * Date: 2018/02/20
 */

// ros headers
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <utility.h>

size_t count = 0;
std::string package_path;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO_STREAM("Point Cloud Received.");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  utility::getPointCloudFromMsg(*msg, *cloud);

  /*
  std::vector<int> indices;
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud < pcl::PointXYZRGBA > temp_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  */

  /*
   * It was found that even though point cloud ros message field says that
   * libfreenect2
   * point cloud is 'rgb', it is 'rgba'. Hence the code below assumes that
   * incoming
   * point cloud is 'rgba' and it converts it to 'rgb' to further use
   */
  /*
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, temp_cloud);
  PointCloudXYZRGBAtoXYZRGB(temp_cloud, *cloud);
  */

  /*
   * point cloud received from libfreenect2 shows that it is dense point cloud
   * which means it shouldn't contain any 'nan' but 'nan' was found. Hence in
   * order to remove 'nan', we first need to make it non-dense. we should
   * make it dense, once 'nan' are removed.
   */
  /*
  cloud->is_dense = false;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  cloud->is_dense = true;
  */

  std::stringstream ss;
  ss << package_path << "/files/scene_" << count << ".pcd";
  std::string file_name = ss.str();

  ROS_INFO_STREAM("Saving point cloud '" << file_name << "'. "
                                         << (cloud->width * cloud->height)
                                         << " Points.");
  pcl::io::savePCDFileASCII(file_name, *cloud);
  count++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_pcd_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");
  std::string cloud_topic;
  nh.getParam("topic", cloud_topic);

  if (cloud_topic.empty())
  {
    cloud_topic = "/kinect1/sd/points";
    ROS_WARN_STREAM("Point cloud topic is not provided. Using '"
                    << cloud_topic << "' as default point cloud topic");
  }
  else
  {
    ROS_INFO_STREAM("Point cloud topic is '" << cloud_topic << "'");
  }

  package_path = ros::package::getPath("multiple_kinect_baxter_calibration");

  ros::Subscriber sub = nh.subscribe(cloud_topic, 1, callback);
  ros::spin();
  return 0;
}
