/**
 * merge_point_clouds.cpp: class file for merging multiple point clouds into one
 * Author: Ravi Joshi
 * Date: 2018/05/14
 */

// ros headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

// tf header
#include <tf/transform_listener.h>

// pcl_ros header
#include <pcl_ros/transforms.h>

// Eigen header
#include <Eigen/Dense>

// we are using 3 kinects
#define KINECT_COUNT 3

class PointCloudSubscriber
{
private:
    Eigen::Matrix4f trans;
    ros::Subscriber subscriber;

    void callback(
        const sensor_msgs::PointCloud2ConstPtr& msg);

    inline void transformPointCloud(
        const Eigen::Matrix4f& transform,
        const sensor_msgs::PointCloud2& in,
        sensor_msgs::PointCloud2& out);

public:
    // the latest available point cloud data
    sensor_msgs::PointCloud2 point_cloud;

    PointCloudSubscriber(){};

    PointCloudSubscriber(
        Eigen::Matrix4f transformation,
        ros::NodeHandle& node_handle,
        std::string topic_name,
        int queue_size);
};

// source: https://github.com/ros-perception/perception_pcl/blob/indigo-devel/pcl_ros/src/transforms.cpp#L106
inline void PointCloudSubscriber::transformPointCloud(
    const Eigen::Matrix4f& transform,
    const sensor_msgs::PointCloud2& in,
    sensor_msgs::PointCloud2& out)
{
    size_t point_step = in.point_step;
    size_t data_size = in.data.size();
    size_t n = in.width * in.height;

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
    Eigen::Vector4f pt_in(0, 0, 0, 1); // 4x1 vector

    for (size_t i = 0; i < n; i++)
    {
        index = i * point_step;

        /*
        * Eigen::Vector4f pt_in (*(float*)&in.data[index + 0],
        *                        *(float*)&in.data[index + 4],
        *                        *(float*)&in.data[index + 8], 1);
        */

        // fill emements of the vector
        pt_in(0) = *(float*)&in.data[index + 0];
        pt_in(1) = *(float*)&in.data[index + 4];
        pt_in(2) = *(float*)&in.data[index + 8];

        // we don't want to make  copy of the data
        pt_out.noalias() = transform * pt_in;

        // copy the data into output
        // we just want to copy first 3 float elements
        std::memcpy(&out.data[index], &pt_out[0], 12);
    }
}

void PointCloudSubscriber::callback(
    const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // pcl_ros::transformPointCloud(trans, *msg, point_cloud);// takes 4 ms
    transformPointCloud(trans, *msg, point_cloud); // takes 3 ms
}

PointCloudSubscriber::PointCloudSubscriber(
    Eigen::Matrix4f transformation,
    ros::NodeHandle& node_handle,
    std::string topic_name,
    int queue_size)
{
    trans = transformation;
    subscriber = node_handle.subscribe<sensor_msgs::PointCloud2>(
        topic_name, queue_size, &PointCloudSubscriber::callback, this,
        ros::TransportHints().tcpNoDelay());
}

class MergePointClouds
{
private:
    std::string base_frame_id, pc1_frame_id, pc2_frame_id, pc3_frame_id;

    Eigen::Matrix4f* fetchTransformations(double wait_time);

    inline void combinePointClouds(
        const sensor_msgs::PointCloud2& cloud1,
        const sensor_msgs::PointCloud2& cloud2,
        const sensor_msgs::PointCloud2& cloud3,
        sensor_msgs::PointCloud2& cloud_out);

public:
    MergePointClouds();
};

// source:
// https://github.com/ros-perception/perception_pcl/blob/118784a4dd06437847182fe5e77829003d103def/pcl_conversions/include/pcl_conversions/pcl_conversions.h#L612
inline void MergePointClouds::combinePointClouds(
    const sensor_msgs::PointCloud2& cloud1,
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
                      cloud2.width * cloud2.height +
                      cloud3.width * cloud3.height;

    cloud_out.is_dense = (!cloud1.is_dense ||
                          !cloud2.is_dense ||
                          !cloud3.is_dense) ? false : true;

    // reserve the memory
    cloud_out.data.resize(cloud1_len + cloud2_len + cloud3_len);

    // copy cloud 2 to output cloud
    std::memcpy(&cloud_out.data[cloud1_len], &cloud2.data[0], cloud2_len);

    // copy cloud 3 to output cloud
    std::memcpy(&cloud_out.data[cloud1_len + cloud2_len], &cloud3.data[0], cloud3_len);
}

Eigen::Matrix4f* MergePointClouds::fetchTransformations(double wait_time)
{
    tf::TransformListener listener;
    tf::StampedTransform t1, t2, t3;

    try
    {
        listener.waitForTransform(base_frame_id, pc1_frame_id, ros::Time(0), ros::Duration(wait_time));
        listener.lookupTransform(base_frame_id, pc1_frame_id, ros::Time(0), t1);
        listener.lookupTransform(base_frame_id, pc2_frame_id, ros::Time(0), t2);
        listener.lookupTransform(base_frame_id, pc3_frame_id, ros::Time(0), t3);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR_STREAM("Unable to fetch static transformations. " << ex.what());
        ros::shutdown();
    }

    // create a dynamic array
    Eigen::Matrix4f *transformations = new Eigen::Matrix4f[KINECT_COUNT];

    tf::StampedTransform tf_trans[] = { t1, t2, t3 };
    for (size_t i = 0; i < KINECT_COUNT; i++)
    {
        Eigen::Matrix4f eigen_transform;
        pcl_ros::transformAsMatrix(tf_trans[i], eigen_transform);
        transformations[i] = eigen_transform;
    }

    return transformations;
}

MergePointClouds::MergePointClouds()
{
    ros::NodeHandle nh("~");

    std::string pc1_topic, pc2_topic, pc3_topic, merge_pc_topic;
    nh.getParam("pc1_topic", pc1_topic);
    nh.getParam("pc2_topic", pc2_topic);
    nh.getParam("pc3_topic", pc3_topic);
    nh.getParam("pc3_topic", pc3_topic);
    nh.getParam("merge_pc_topic", merge_pc_topic);

    nh.getParam("base_frame_id", base_frame_id);
    nh.getParam("pc1_frame_id", pc1_frame_id);
    nh.getParam("pc2_frame_id", pc2_frame_id);
    nh.getParam("pc3_frame_id", pc3_frame_id);

    double wait_time;
    nh.getParam("wait_time", wait_time);

    int freq;
    nh.getParam("freq", freq);

    Eigen::Matrix4f *transformations = fetchTransformations(wait_time);

    PointCloudSubscriber pc_sub1(transformations[0], nh, pc1_topic, 1);
    PointCloudSubscriber pc_sub2(transformations[1], nh, pc2_topic, 1);
    PointCloudSubscriber pc_sub3(transformations[2], nh, pc3_topic, 1);

    ros::Publisher merged_pc_pub = nh.advertise<sensor_msgs::PointCloud2>(merge_pc_topic, 1);
    ros::Rate loop_rate(freq);

    while (ros::ok())
    {
        sensor_msgs::PointCloud2 cloud_out;
        combinePointClouds(pc_sub1.point_cloud,
                           pc_sub2.point_cloud,
                           pc_sub3.point_cloud,
                           cloud_out);

        // we need to set header before publishing it
        cloud_out.header.frame_id = base_frame_id;
        cloud_out.header.stamp = ros::Time::now();

        merged_pc_pub.publish(cloud_out);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "merge_point_clouds", ros::init_options::AnonymousName);
    MergePointClouds merge_point_clouds;

    return 0;
}
