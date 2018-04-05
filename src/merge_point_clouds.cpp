/**
* merge_point_clouds.cpp
* Author: Ravi Joshi
* Date: 2018/04/05
*/

#include <ros/ros.h>
#include <utility.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <message_filters/sync_policies/approximate_time.h>

class MergePointClouds
{
    ros::Publisher merge_pc_pub;
    tf::StampedTransform transformations[3];
    std::string base_frame_id, pc1_frame_id, pc2_frame_id, pc3_frame_id;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> pc_filter;

    void initFilter(int nr_k, double stddev_mult);
    void fetchTransformations(tf::StampedTransform transformations[]);
    void filterPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in, pcl::PointCloud<pcl::PointXYZRGB> &out);
    void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg1, const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg2, const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg3);
  public:
    MergePointClouds();
};


void MergePointClouds::filterPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in, pcl::PointCloud<pcl::PointXYZRGB> &out)
{
    pc_filter.setInputCloud(in);
    pc_filter.filter(out);
}

void MergePointClouds::initFilter(int nr_k, double stddev_mult)
{
    pc_filter.setMeanK(nr_k);
    pc_filter.setStddevMulThresh(stddev_mult);
}

void MergePointClouds::callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg1, const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg2, const boost::shared_ptr<const sensor_msgs::PointCloud2>& pc_msg3)
{
    ROS_DEBUG_STREAM("MergePointClouds callback received");

    pcl::PointCloud<pcl::PointXYZRGB> cloud1, cloud2, cloud3;

    utility::getPointCloudFromMsg(pc_msg1, cloud1);
    utility::getPointCloudFromMsg(pc_msg2, cloud2);
    utility::getPointCloudFromMsg(pc_msg3, cloud3);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr t1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr t2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl_ros::transformPointCloud(cloud1, *raw_cloud, transformations[0]);
    pcl_ros::transformPointCloud(cloud2, *t1, transformations[1]);
    pcl_ros::transformPointCloud(cloud3, *t2, transformations[2]);

    *raw_cloud += *t1;
    *raw_cloud += *t2;

    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    ros::Time now = ros::Time::now();
    cloud.header.frame_id = base_frame_id;

    // https://answers.ros.org/question/172730/pcl-header-timestamp/
    pcl_conversions::toPCL(now, cloud.header.stamp);

    // Statistical outlier removal filtering
    filterPointCloud(raw_cloud, cloud);

    merge_pc_pub.publish(cloud);
}

void MergePointClouds::fetchTransformations(tf::StampedTransform transformations[])
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    tf::StampedTransform tf_t1, tf_t2, tf_t3;
    geometry_msgs::TransformStamped gm_t1, gm_t2, gm_t3;

    try
    {
        // get the latest available transformations
        gm_t1 = tfBuffer.lookupTransform("base", "kinect1_link", ros::Time(0));
        //gm_t2 = tfBuffer.lookupTransform(base_frame_id, pc2_frame_id, ros::Time(0));
        //gm_t3 = tfBuffer.lookupTransform(base_frame_id, pc3_frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR_STREAM("Unable to fetch static transformations. " << ex.what());
        ros::shutdown();
    }

    // convert transformations to tf
    tf::transformStampedMsgToTF(gm_t1, tf_t1);
    tf::transformStampedMsgToTF(gm_t2, tf_t2);
    tf::transformStampedMsgToTF(gm_t3, tf_t3);

    transformations[0] = tf_t1; transformations[1] = tf_t2; transformations[2] = tf_t3;
}

MergePointClouds::MergePointClouds()
{
    ros::NodeHandle nh("~");

    std::string pc1_topic, pc2_topic, pc3_topic;
    nh.getParam("pc1_topic", pc1_topic);
    nh.getParam("pc2_topic", pc2_topic);
    nh.getParam("pc3_topic", pc3_topic);

    nh.getParam("base_frame_id", base_frame_id);
    nh.getParam("pc1_frame_id", pc1_frame_id);
    nh.getParam("pc2_frame_id", pc2_frame_id);
    nh.getParam("pc3_frame_id", pc3_frame_id);

    merge_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("merge_points", 1);

    int nr_k;
    double stddev_mult;
    nh.getParam("nr_k", nr_k);
    nh.getParam("stddev_mult", stddev_mult);
    initFilter(nr_k, stddev_mult);

    // before proceeding further we should get the calibration information
    fetchTransformations(transformations);

    message_filters::Subscriber<sensor_msgs::PointCloud2> pc1_sub(nh, pc1_topic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc2_sub(nh, pc2_topic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc3_sub(nh, pc3_topic, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), pc1_sub, pc2_sub, pc3_sub);
    sync.registerCallback(boost::bind(&MergePointClouds::callback, this, _1, _2, _3));

    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "merge_point_clouds", ros::init_options::AnonymousName);
    MergePointClouds merge_point_clouds;

    return 0;
}
