/**
 * show_all_pc.cpp: class file for visualizing the contribution of each
 * kinect sensor
 * Author: Ravi Joshi
 * Date: 2018/04/04
 */

#include <utility.h>

// ros headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

class ShowAllPC
{
  ros::Publisher pc1_pub, pc2_pub, pc3_pub;
  uint32_t red_color, green_color, blue_color;
  std::string pc1_frame_id, pc2_frame_id, pc3_frame_id;

  uint32_t packRGB(uint8_t r, uint8_t g, uint8_t b);

  void applyColor(pcl::PointCloud<pcl::PointXYZRGB>& cloud, uint32_t color);

  void
  callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg1,
           const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg2,
           const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg3);

public:
  ShowAllPC();
};

uint32_t ShowAllPC::packRGB(uint8_t r, uint8_t g, uint8_t b)
{
  // src: http://docs.pointclouds.org/1.7.0/structpcl_1_1_point_x_y_z_r_g_b.html
  uint32_t color = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  return color;
}

void ShowAllPC::applyColor(pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                           uint32_t color)
{
  for (size_t i = 0; i < cloud.points.size(); i++)
    cloud.points[i].rgb = *reinterpret_cast<float*>(&color);
}

void ShowAllPC::callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg1,
                         const sensor_msgs::PointCloud2ConstPtr& pc_msg2,
                         const sensor_msgs::PointCloud2ConstPtr& pc_msg3)
{
  ROS_DEBUG_STREAM("ShowAllPC callback received");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  utility::getPointCloudFromMsg(*pc_msg1, *cloud1);
  utility::getPointCloudFromMsg(*pc_msg2, *cloud2);
  utility::getPointCloudFromMsg(*pc_msg3, *cloud3);

  applyColor(*cloud1, red_color);
  applyColor(*cloud2, green_color);
  applyColor(*cloud3, blue_color);

  cloud1->header.frame_id = pc1_frame_id;
  cloud2->header.frame_id = pc2_frame_id;
  cloud3->header.frame_id = pc3_frame_id;

  ros::Time now = ros::Time::now();

  // https://answers.ros.org/question/172730/pcl-header-timestamp/
  pcl_conversions::toPCL(now, cloud1->header.stamp);
  pcl_conversions::toPCL(now, cloud2->header.stamp);
  pcl_conversions::toPCL(now, cloud3->header.stamp);

  pc1_pub.publish(cloud1);
  pc2_pub.publish(cloud2);
  pc3_pub.publish(cloud3);
}

ShowAllPC::ShowAllPC()
{
  ros::NodeHandle nh("~");

  std::string pc1_topic, pc2_topic, pc3_topic;
  nh.getParam("pc1_topic", pc1_topic);
  nh.getParam("pc2_topic", pc2_topic);
  nh.getParam("pc3_topic", pc3_topic);

  nh.getParam("pc1_frame_id", pc1_frame_id);
  nh.getParam("pc2_frame_id", pc2_frame_id);
  nh.getParam("pc3_frame_id", pc3_frame_id);

  red_color = packRGB(191, 63, 63);
  green_color = packRGB(63, 191, 63);
  blue_color = packRGB(63, 63, 191);

  pc1_pub = nh.advertise<sensor_msgs::PointCloud2>("pc_1_points", 1);
  pc2_pub = nh.advertise<sensor_msgs::PointCloud2>("pc_2_points", 1);
  pc3_pub = nh.advertise<sensor_msgs::PointCloud2>("pc_3_points", 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> pc1_sub(nh, pc1_topic,
                                                                1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc2_sub(nh, pc2_topic,
                                                                1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc3_sub(nh, pc3_topic,
                                                                1);

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2>
      SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), pc1_sub,
                                                 pc2_sub, pc3_sub);
  sync.registerCallback(boost::bind(&ShowAllPC::callback, this, _1, _2, _3));

  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "show_all_point_cloud_node",
            ros::init_options::AnonymousName);
  ShowAllPC show_all_point_cloud;

  return 0;
}
