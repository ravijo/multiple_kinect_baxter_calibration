/**
 * save_pcd.cpp: utility to save point cloud data to file
 * Author: Ravi Joshi
 * Date: 2018/02/20
 */

// ros headers
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <utility.h>

std::string package_path;

/*
int file_index = 0;
std::string saveDir = "/files/captured_pcd";
std::string pointCloudTopic = "/kinect1/sd/points";

inline void PointCloudXYZRGBAtoXYZRGB(pcl::PointCloud<pcl::PointXYZRGBA>& in,
    pcl::PointCloud<pcl::PointXYZRGB>& out) {
  out.width = in.width;
  out.height = in.height;
  out.points.resize(in.points.size());
  for (size_t i = 0; i < in.points.size(); i++) {
    out.points[i].x = in.points[i].x;
    out.points[i].y = in.points[i].y;
    out.points[i].z = in.points[i].z;
    out.points[i].r = in.points[i].r;
    out.points[i].g = in.points[i].g;
    out.points[i].b = in.points[i].b;
  }
}
*/

void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg) {
  ROS_INFO("Point Cloud Received.");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  utility::getPointCloudFromMsg(msg, *cloud);

  /*
  std::vector<int> indices;
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud < pcl::PointXYZRGBA > temp_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  */

  /*
   * It was found that even though point cloud ros message field says that libfreenect2
   * point cloud is 'rgb', it is 'rgba'. Hence the code below assumes that incoming
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

  std::string file_name = package_path + "/files/scene.pcd";
  ROS_INFO_STREAM("Saving point cloud '" << file_name << "'. " << (cloud->width * cloud->height) << " Points.");
  pcl::io::savePCDFileASCII(file_name, *cloud);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "save_pcd", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");
  std::string cloud_topic;
  nh.getParam("topic", cloud_topic);

  if(cloud_topic.empty()){
    cloud_topic = "/kinect1/sd/points";
    ROS_WARN_STREAM("Point cloud topic is not provided. Using '" << cloud_topic << "' as default point cloud topic");
  }else{
    ROS_INFO_STREAM("Point cloud topic is '" << cloud_topic << "'");
  }

  package_path = ros::package::getPath("multiple_kinect_baxter_calibration");

  ros::Subscriber sub = nh.subscribe(cloud_topic, 1, callback);
  ros::spin();
  return 0;
}
