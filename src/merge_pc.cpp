/**
 * merge_pc.cpp: class file for merging multiple point clouds into one
 * Author: Ravi Joshi
 * Date: 2018/05/14
 */

// ros headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>

// tf header
#include <tf/transform_listener.h>

// pcl_ros header
#include <pcl_ros/transforms.h>

// Eigen header
#include <Eigen/Dense>

// for thread related support
#include <boost/thread.hpp>

// for locking using mutex
#include <pthread.h>

// we are using 3 kinects
#define KINECT_COUNT 3

pthread_mutex_t my_mutex;

class PointCloudSubscriber
{
private:
  Eigen::Matrix4f trans;
  ros::Subscriber subscriber;
  sensor_msgs::PointCloud2 temp_cloud;

  void callback(const sensor_msgs::PointCloud2ConstPtr& msg);

  inline void transformPointCloud(const Eigen::Matrix4f& transform,
                                  const sensor_msgs::PointCloud2& in,
                                  sensor_msgs::PointCloud2& out);

public:
  // the latest available point cloud data
  sensor_msgs::PointCloud2 point_cloud;

  void setup(Eigen::Matrix4f transformation, ros::NodeHandle& node_handle,
             std::string topic_name, int queue_size);

  PointCloudSubscriber(){};
};

// source:
// https://github.com/ros-perception/perception_pcl/blob/indigo-devel/pcl_ros/src/transforms.cpp#L106
inline void
PointCloudSubscriber::transformPointCloud(const Eigen::Matrix4f& transform,
                                          const sensor_msgs::PointCloud2& in,
                                          sensor_msgs::PointCloud2& out)
{
  const size_t point_step = in.point_step;
  const size_t data_size = in.data.size();
  const size_t n = in.width * in.height;

  // copy the other data
  out.header = in.header;
  out.height = in.height;
  out.width = in.width;
  out.fields = in.fields;
  out.is_bigendian = in.is_bigendian;
  out.point_step = point_step;
  out.row_step = in.row_step;
  out.is_dense = in.is_dense;
  out.data.resize(data_size);

  // copy complete point cloud data since it contain colors as well
  std::memcpy(&out.data[0], &in.data[0], data_size);

  size_t index;
  Eigen::Vector4f pt_out;
  Eigen::Vector4f pt_in(0, 0, 0, 1);  // 4x1 vector
  for (size_t i = 0; i < n; i++)
  {
    index = i * point_step;
    /*
     * Eigen::Vector4f pt_in (*(float*)&in.data[index + 0],
     *                        *(float*)&in.data[index + 4],
     *                        *(float*)&in.data[index + 8], 1);
     */

    // fill emements of the vector by first 3 float elements (12 bytes)
    std::memcpy(&pt_in, &in.data[index], 12);

    // perform the transformation here
    // we don't want to make  copy of the data hence using noalias()
    pt_out.noalias() = transform * pt_in;

    // copy the data into output
    // we just want to copy first 3 float elements (12 bytes)
    std::memcpy(&out.data[index], &pt_out[0], 12);
  }
}

void PointCloudSubscriber::callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // we work with a temporary cloud i.e., temp_cloud once temp_cloud is
  // transformed it is simply assigned to point_cloud
  transformPointCloud(trans, *msg, temp_cloud);

  // make sure that other threads cannot access the point cloud while we are
  // assigning it
  pthread_mutex_lock(&my_mutex);
  point_cloud = temp_cloud;
  pthread_mutex_unlock(&my_mutex);
}

void PointCloudSubscriber::setup(Eigen::Matrix4f transformation,
                                 ros::NodeHandle& node_handle,
                                 std::string topic_name, int queue_size)
{
  trans = transformation;
  subscriber = node_handle.subscribe<sensor_msgs::PointCloud2>(
      topic_name, queue_size, &PointCloudSubscriber::callback, this,
      ros::TransportHints().tcpNoDelay());
}

class MergePC
{
private:
  int freq;
  std::string base_frame_id, pc1_frame_id, pc2_frame_id, pc3_frame_id;
  PointCloudSubscriber pc_sub1, pc_sub2, pc_sub3;
  ros::Publisher merged_pc_pub;
  boost::shared_ptr<ros::AsyncSpinner> spinner;

  Eigen::Matrix4f* fetchTransformations(double wait_time);

  inline void combinePointClouds(const sensor_msgs::PointCloud2& cloud1,
                                 const sensor_msgs::PointCloud2& cloud2,
                                 const sensor_msgs::PointCloud2& cloud3,
                                 sensor_msgs::PointCloud2& cloud_out);

public:
  MergePC();
  void run();
};

// source:
// https://github.com/ros-perception/perception_pcl/blob/118784a4dd06437847182fe5e77829003d103def/pcl_conversions/include/pcl_conversions/pcl_conversions.h#L612
inline void MergePC::combinePointClouds(const sensor_msgs::PointCloud2& cloud1,
                                        const sensor_msgs::PointCloud2& cloud2,
                                        const sensor_msgs::PointCloud2& cloud3,
                                        sensor_msgs::PointCloud2& cloud_out)
{
  // assign cloud 1 to output cloud
  cloud_out = cloud1;

  // get the lengths of each cloud
  size_t cloud1_len = cloud1.data.size();
  size_t cloud2_len = cloud2.data.size();
  size_t cloud3_len = cloud3.data.size();

  // set the parameters of output cloud
  cloud_out.height = 1;
  cloud_out.width = cloud1.width * cloud1.height +
                    cloud2.width * cloud2.height + cloud3.width * cloud3.height;

  cloud_out.is_dense =
      (!cloud1.is_dense || !cloud2.is_dense || !cloud3.is_dense) ? false : true;

  // reserve the memory
  cloud_out.data.resize(cloud1_len + cloud2_len + cloud3_len);

  // copy cloud 2 to output cloud
  std::memcpy(&cloud_out.data[cloud1_len], &cloud2.data[0], cloud2_len);

  // copy cloud 3 to output cloud
  std::memcpy(&cloud_out.data[cloud1_len + cloud2_len], &cloud3.data[0],
              cloud3_len);
}

Eigen::Matrix4f* MergePC::fetchTransformations(double wait_time)
{
  tf::TransformListener listener;
  tf::StampedTransform t1, t2, t3;

  try
  {
    listener.waitForTransform(base_frame_id, pc1_frame_id, ros::Time(0),
                              ros::Duration(wait_time));
    listener.lookupTransform(base_frame_id, pc1_frame_id, ros::Time(0), t1);
    listener.lookupTransform(base_frame_id, pc2_frame_id, ros::Time(0), t2);
    listener.lookupTransform(base_frame_id, pc3_frame_id, ros::Time(0), t3);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR_STREAM("Unable to fetch static transformations. " << ex.what());
    ros::shutdown();
  }

  // create the array on heap
  Eigen::Matrix4f* transformations = new Eigen::Matrix4f[KINECT_COUNT];

  tf::StampedTransform tf_trans[] = { t1, t2, t3 };
  for (size_t i = 0; i < KINECT_COUNT; i++)
  {
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(tf_trans[i], eigen_transform);
    transformations[i] = eigen_transform;
  }

  return transformations;
}

MergePC::MergePC()
{
  ros::NodeHandle nh("~");

  std::string pc1_topic, pc2_topic, pc3_topic, merge_pc_topic;
  nh.getParam("pc1_topic", pc1_topic);
  nh.getParam("pc2_topic", pc2_topic);
  nh.getParam("pc3_topic", pc3_topic);
  nh.getParam("merge_pc_topic", merge_pc_topic);

  nh.getParam("base_frame_id", base_frame_id);
  nh.getParam("pc1_frame_id", pc1_frame_id);
  nh.getParam("pc2_frame_id", pc2_frame_id);
  nh.getParam("pc3_frame_id", pc3_frame_id);

  double wait_time;
  nh.getParam("wait_time", wait_time);
  nh.getParam("freq", freq);

  Eigen::Matrix4f* transformations = fetchTransformations(wait_time);
  pc_sub1.setup(transformations[0], nh, pc1_topic, 1);
  pc_sub2.setup(transformations[1], nh, pc2_topic, 1);
  pc_sub2.setup(transformations[2], nh, pc3_topic, 1);

  // create a publisher for publishing merged point cloud
  merged_pc_pub = nh.advertise<sensor_msgs::PointCloud2>(merge_pc_topic, 1);

  // get the concurrent hardware thread count
  const int spported_threads = boost::thread::hardware_concurrency();
  ROS_DEBUG_STREAM("Number of heardware supported threads for this CPU is "
                   << spported_threads);

  // we need one thread for each kinect
  const int desired_threads = KINECT_COUNT;

  // show warning to user
  if (desired_threads > spported_threads)
    ROS_WARN_STREAM(
        "It is suggested to upgrade the CPU for better performance");
  else
    ROS_DEBUG_STREAM("Initializing " << desired_threads
                                     << " additional threads");

  // lazy initialization for boost::shared_ptr
  // source: https://stackoverflow.com/a/12997218/1175065
  spinner.reset(new ros::AsyncSpinner(desired_threads));
}

void MergePC::run()
{
  // make sure we have the data. so we need to start the spinner
  spinner->start();

  ros::Rate loop_rate(freq);
  sensor_msgs::PointCloud2 cloud_out;

  while (ros::ok())
  {
    combinePointClouds(pc_sub1.point_cloud, pc_sub2.point_cloud,
                       pc_sub3.point_cloud, cloud_out);

    // we need to set header before publishing it
    cloud_out.header.frame_id = base_frame_id;
    cloud_out.header.stamp = ros::Time::now();
    merged_pc_pub.publish(cloud_out);

    loop_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "merge_point_clouds_node",
            ros::init_options::AnonymousName);

  MergePC merge_point_clouds;
  merge_point_clouds.run();

  return 0;
}
