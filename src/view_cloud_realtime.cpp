/**
 * view_cloud_realtime.cpp: utility to view point cloud data in realtime
 * Author: Ravi Joshi
 * Date: 2018/02/20
 */

// ros headers
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <utility.h>

// pcl header
#include <pcl/visualization/pcl_visualizer.h>


pcl::visualization::PCLVisualizer viewer("Realtime Cloud Viewer");

void callback(
    const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  utility::getPointCloudFromMsg(msg, *cloud);

  if (!viewer.updatePointCloud(cloud, "cloud"))
    viewer.addPointCloud(cloud, "cloud");

  viewer.spinOnce();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "view_cloud_realtime_node",
      ros::init_options::AnonymousName);

  ros::NodeHandle nh("~");
  std::string cloud_topic;
  nh.getParam("topic", cloud_topic);

  if (cloud_topic.empty()) {
    cloud_topic = "/kinect1/sd/points";
    ROS_WARN_STREAM(
        "Point cloud topic is not provided. Using '" << cloud_topic
            << "' as default point cloud topic");
  } else {
    ROS_INFO_STREAM("Point cloud topic is '" << cloud_topic << "'");
  }
  std::string package_path =
  ros::package::getPath("multiple_kinect_baxter_calibration");

  std::string cam_file;
  std::string source;
  if (nh.getParam("source", source) && boost::starts_with(boost::algorithm::to_lower_copy(source), "w")) {
    // if source is 'Windows'
    cam_file = package_path + "/files/kinect_anywhere.cam";
  } else {
    // if source is 'Linux'
    cam_file = package_path + "/files/libfreenect.cam";
  }
  ROS_INFO_STREAM("cam_file is '" << cam_file << "'");

  pcl::visualization::Camera camera;
  std::vector<std::string> cam_param;
  bool result = utility::loadCameraParametersPCL(cam_file, cam_param);
  result = result && utility::getCameraParametersPCL(cam_param, camera);
  ROS_DEBUG_STREAM("loadCameraParametersPCL returned " << result);

  ros::Subscriber sub = nh.subscribe(cloud_topic, 1, callback);

  viewer.initCameraParameters();
  viewer.setCameraParameters(camera);

  ros::spin();

  return 0;
}
