/**
 * merge_point_clouds.cpp: class file for merging multiple point clouds into one
 * Author: Ravi Joshi
 * Date: 2018/04/05
 */

// utility header
#include <utility.h>

// ros headers
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

// tf header
#include <tf/transform_listener.h>

// pcl header
#include <pcl/filters/statistical_outlier_removal.h>

// for time measurement
#include <sys/time.h>

// maximum attempts for tf in order to fetch transformations
#define TF_MAX_ATTEMPTS 5

class MergePointClouds {
  struct timeval tp;

  bool still_processing;
  ros::Publisher merge_pc_pub;
  tf::StampedTransform transformations[3];
  std::string base_frame_id, pc1_frame_id, pc2_frame_id, pc3_frame_id;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> pc_filter;

  void initFilter(int nr_k, double stddev_mult);

  void fetchTransformations(double wait_time,
      tf::StampedTransform transformations[]);

  void filterPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
      pcl::PointCloud<pcl::PointXYZRGB> &out);

  void callback(
      const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg1,
      const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg2,
      const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg3);
public:
  MergePointClouds();
};

void MergePointClouds::filterPointCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
    pcl::PointCloud<pcl::PointXYZRGB> &out) {
  pc_filter.setInputCloud(in);
  pc_filter.filter(out);
}

void MergePointClouds::initFilter(int nr_k, double stddev_mult) {
  pc_filter.setMeanK(nr_k);
  pc_filter.setStddevMulThresh(stddev_mult);
}

void MergePointClouds::callback(
    const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg1,
    const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg2,
    const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg3) {

  gettimeofday(&tp, NULL);
  long int t1 = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  //printf("Time (94828 us=94.828 ms) %lu us\n", tp.tv_usec);

  //return;

  /*
  if (still_processing)
    return;
  still_processing = true;
  */

  //ROS_DEBUG_STREAM("MergePointClouds callback received");

  pcl::PointCloud<pcl::PointXYZRGB> cloud1, cloud2, cloud3;

  utility::getPointCloudFromMsg(pc_msg1, cloud1);
  utility::getPointCloudFromMsg(pc_msg2, cloud2);
  utility::getPointCloudFromMsg(pc_msg3, cloud3);

  gettimeofday(&tp, NULL);
  long int t2 = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  std::cout << "utility::getPointCloudFromMsg took (ms) " << (t2 - t1) << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp1(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp2(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl_ros::transformPointCloud(cloud1, *raw_cloud, transformations[0]);
  pcl_ros::transformPointCloud(cloud2, *temp1, transformations[1]);
  pcl_ros::transformPointCloud(cloud3, *temp2, transformations[2]);

  gettimeofday(&tp, NULL);
  long int t3 = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  std::cout << "pcl_ros::transformPointCloud took (ms) " << (t3 - t2) << std::endl;

  *raw_cloud += *temp1;
  *raw_cloud += *temp2;

  gettimeofday(&tp, NULL);
  long int t4 = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  std::cout << "PCL concatination took (ms) " << (t4 - t3) << std::endl;

  pcl::PointCloud < pcl::PointXYZRGB > cloud;

  // Statistical outlier removal filtering
  filterPointCloud(raw_cloud, cloud);

  gettimeofday(&tp, NULL);
  long int t5 = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  std::cout << "filterPointCloud took (ms) " << (t5 - t4) << std::endl;

  ros::Time now = ros::Time::now();
  cloud.header.frame_id = base_frame_id;

  // https://answers.ros.org/question/172730/pcl-header-timestamp/
  pcl_conversions::toPCL(now, cloud.header.stamp);
  gettimeofday(&tp, NULL);
  long int t6 = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  std::cout << "pcl_conversions::toPCL took (ms) " << (t6 - t5) << std::endl;

  merge_pc_pub.publish(cloud);

  //still_processing = false;

  gettimeofday(&tp, NULL);
  long int t7 = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  std::cout << "publish clous took (ms) " << (t7 - t6) << std::endl;
}

void MergePointClouds::fetchTransformations(double wait_time,
    tf::StampedTransform transformations[]) {
  tf::TransformListener listener;
  tf::StampedTransform t1, t2, t3;

  try {
    listener.waitForTransform(base_frame_id, pc1_frame_id, ros::Time(0),
        ros::Duration(wait_time));
    listener.lookupTransform(base_frame_id, pc1_frame_id, ros::Time(0), t1);
    listener.lookupTransform(base_frame_id, pc2_frame_id, ros::Time(0), t2);
    listener.lookupTransform(base_frame_id, pc3_frame_id, ros::Time(0), t3);
  } catch (tf::TransformException ex) {
    ROS_ERROR_STREAM(
        "Unable to fetch static transformations. " << ex.what());
    ros::shutdown();
  }

  transformations[0] = t1;
  transformations[1] = t2;
  transformations[2] = t3;
}

MergePointClouds::MergePointClouds() {
  ros::NodeHandle nh("~");

  std::string pc1_topic, pc2_topic, pc3_topic;
  nh.getParam("pc1_topic", pc1_topic);
  nh.getParam("pc2_topic", pc2_topic);
  nh.getParam("pc3_topic", pc3_topic);

  nh.getParam("base_frame_id", base_frame_id);
  nh.getParam("pc1_frame_id", pc1_frame_id);
  nh.getParam("pc2_frame_id", pc2_frame_id);
  nh.getParam("pc3_frame_id", pc3_frame_id);

  merge_pc_pub = nh.advertise < sensor_msgs::PointCloud2
      > ("merge_points", 5);

  int nr_k;
  double stddev_mult;
  nh.getParam("nr_k", nr_k);
  nh.getParam("stddev_mult", stddev_mult);
  initFilter(nr_k, stddev_mult);

  double wait_time;
  nh.getParam("wait_time", wait_time);

  // before proceeding further we should get the calibration information
  fetchTransformations(wait_time, transformations);

  // we haven't started processing yet
  still_processing = false;

  message_filters::Subscriber < sensor_msgs::PointCloud2
      > pc1_sub(nh, pc1_topic, 1);
  message_filters::Subscriber < sensor_msgs::PointCloud2
      > pc2_sub(nh, pc2_topic, 1);
  message_filters::Subscriber < sensor_msgs::PointCloud2
      > pc3_sub(nh, pc3_topic, 1);

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(100), pc1_sub,
      pc2_sub, pc3_sub);
  sync.registerCallback(
      boost::bind(&MergePointClouds::callback, this, _1, _2, _3));

  ros::spin();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "merge_point_clouds",
      ros::init_options::AnonymousName);
  MergePointClouds merge_point_clouds;

  return 0;
}
