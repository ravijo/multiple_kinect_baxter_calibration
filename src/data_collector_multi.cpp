/**
* data_collector_multi.cpp: class file for collecting data using multiple kinects
* Author: Ravi Joshi
* Date: 2018/02/28
*/

#include <vector>
#include <ros/ros.h>
#include <utility.h>
#include <Eigen/Geometry>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <sphere_detector.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <baxter_core_msgs/EndpointState.h>
#include <message_filters/sync_policies/approximate_time.h>

// number of kinects used
#define KINECT_COUNT 3

// baxter arm motion state (moving:0, stop:1, finished:2)
#define MOVING 0
#define STOP 1
#define FINISHED 2

class DataCollectorMulti
{
  private:
    int frame;
    int baxter_arm_motion_state; // moving:0, stop:1, finished:2
    Eigen::Matrix4d t_ball_wrt_ee;
    std::string ee_topic, data_dir;
    std_msgs::Bool still_processing;
    pcl::visualization::Camera camera;
    std::vector<std::string> pc_topics;
    ros::Subscriber baxter_arm_motion_status_sub;
    pcl_project::SphereDetector* sphere_detector;
    std::vector<boost::shared_ptr<pcl::visualization::PCLVisualizer> > pc_viewers;
    ros::Publisher data_collection_progress_pub;
    std::vector<std::vector<float> > position_wrt_baxter;
    std::vector<std::vector<std::vector<float> > > position_wrt_kinects;

    void saveData();
    void init(ros::NodeHandle nh);
    void initPCViewers(std::string cam_file);
    void baxterArmMotionStatusCallback(const std_msgs::Int8::ConstPtr& msg);
    void recordBallPositionWrtKinect(pcl::ModelCoefficients spheres_coeff[]);
    void recordBallPositionWrtBaxter(baxter_core_msgs::EndpointStateConstPtr ee_msg);
    void callback(const baxter_core_msgs::EndpointStateConstPtr& ee_msg, const sensor_msgs::PointCloud2ConstPtr& pc1_msg, const sensor_msgs::PointCloud2ConstPtr& pc2_msg, const sensor_msgs::PointCloud2ConstPtr& pc3_msg);
  public:
    DataCollectorMulti();
};

void DataCollectorMulti::recordBallPositionWrtBaxter(baxter_core_msgs::EndpointStateConstPtr ee_msg)
{
    Eigen::Quaterniond q(ee_msg->pose.orientation.w, ee_msg->pose.orientation.x, ee_msg->pose.orientation.y, ee_msg->pose.orientation.z);

    Eigen::Matrix4d t_ee_wrt_base;
    t_ee_wrt_base.setIdentity(); // identity to make bottom row of matrix 0, 0, 0, 1
    t_ee_wrt_base.block<3, 3>(0, 0) = q.toRotationMatrix();
    t_ee_wrt_base(0, 3) = ee_msg->pose.position.x;
    t_ee_wrt_base(1, 3) = ee_msg->pose.position.y;
    t_ee_wrt_base(2, 3) = ee_msg->pose.position.z;

    Eigen::Matrix4d t = t_ee_wrt_base * t_ball_wrt_ee;

    std::vector<float> baxter;
    for (int i = 0; i < 3; i++)
        baxter.push_back(t(i, 3));

    position_wrt_baxter.push_back(baxter);
}

void DataCollectorMulti::recordBallPositionWrtKinect(pcl::ModelCoefficients spheres_coeff[])
{
    for (size_t kinect_id = 0; kinect_id < KINECT_COUNT; kinect_id++)
    {
        std::vector<float> ball_center;
        for (int i = 0; i < 3; i++)
            ball_center.push_back(spheres_coeff[kinect_id].values[i]);

        position_wrt_kinects.at(kinect_id).push_back(ball_center);
    }
}

void DataCollectorMulti::baxterArmMotionStatusCallback(const std_msgs::Int8::ConstPtr& msg)
{
    baxter_arm_motion_state = msg->data;
}

void DataCollectorMulti::saveData()
{
    std::string header = "position_x,position_y,position_z";
    std::string baxter = data_dir + "/position_wrt_baxter_multiple_kinect.csv";

    ROS_INFO_STREAM("Saving collected data in following files: " << baxter << " and " << "kinect");

    bool status;
    std::string err;

    status = utility::writeCSV(baxter, header, position_wrt_baxter, err);
    if(!status) ROS_ERROR_STREAM("Unable to save tracking data of baxter to CSV file. " << err);

    for (size_t kinect_id = 0; kinect_id < KINECT_COUNT; kinect_id++)
    {
      std::string kinect_file = data_dir + "/position_wrt_multiple_kinect_" + utility::to_string(kinect_id + 1) + ".csv";
      status = utility::writeCSV(kinect_file, header, position_wrt_kinects.at(kinect_id), err);
      if(!status) ROS_ERROR_STREAM("Unable to save tracking data of kinect " << kinect_id << " to CSV file. " << err);
    }
}

void DataCollectorMulti::callback(const baxter_core_msgs::EndpointStateConstPtr& ee_msg, const sensor_msgs::PointCloud2ConstPtr& pc1_msg, const sensor_msgs::PointCloud2ConstPtr& pc2_msg, const sensor_msgs::PointCloud2ConstPtr& pc3_msg)
{
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds[KINECT_COUNT];
      for (size_t i = 0; i < KINECT_COUNT; i++)
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        clouds[i] = cloud;
      }

      //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
      //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZRGB>);

      utility::getPointCloudFromMsg(pc1_msg, *clouds[0]);
      utility::getPointCloudFromMsg(pc2_msg, *clouds[1]);
      utility::getPointCloudFromMsg(pc3_msg, *clouds[2]);

    // don't proceed further if robot is moving or if we are processing the existing data
    if (still_processing.data || (baxter_arm_motion_state == MOVING)) return;

    still_processing.data = true; // we are about to start processing
    data_collection_progress_pub.publish(still_processing);

    //ROS_INFO_STREAM("Cloud 1 " << clouds[0]->points.size() << ", Cloud 2 "  << clouds[1]->points.size() << ", Cloud 3 "  << clouds[2]->points.size() );

    //pcl::io::savePCDFileASCII ("/home/tom/Documents/ravi/Recycle_Bin/del/1.pcd", *clouds[0]);
    //pcl::io::savePCDFileASCII ("/home/tom/Documents/ravi/Recycle_Bin/del/2.pcd", *clouds[1]);
    //pcl::io::savePCDFileASCII ("/home/tom/Documents/ravi/Recycle_Bin/del/3.pcd", *clouds[2]);

/*
    // show caputed point cloud
    for (size_t i = 0; i < KINECT_COUNT; i++)
    {
        std::string cloud_id = "cloud" + utility::to_string(i);
        if (!pc_viewers.at(i)->updatePointCloud(clouds[i], cloud_id))
            pc_viewers.at(i)->addPointCloud(clouds[i], cloud_id);

        //std::string sphere_id = "detected_sphere" + utility::to_string(i);
        //pc_viewers.at(i)->removeShape(sphere_id);

        // force the viewer to show updaded cloud
        pc_viewers.at(i)->setCameraParameters(camera);
        pc_viewers.at(i)->spinOnce();
    }
*/
    /*
    if (!pc_viewers.at(1)->updatePointCloud(clouds[1], "cloud1"))
        pc_viewers.at(1)->addPointCloud(clouds[1], "cloud1");

        */

    /*
    if (!pc_viewers.at(1)->updatePointCloud(cloud2, "cloud2"))
        pc_viewers.at(1)->addPointCloud(cloud2, "cloud2");

    if (!pc_viewers.at(2)->updatePointCloud(cloud3, "cloud3"))
        pc_viewers.at(2)->addPointCloud(cloud3, "cloud3");
    */

    // exit and save the data
    if (baxter_arm_motion_state == FINISHED)
    {
      if(position_wrt_baxter.empty())
        ROS_ERROR_STREAM("Couldn't record any data. Run the program again.");
      else
        saveData();

      ROS_INFO_STREAM("Exiting now...");
      ros::shutdown();
      return;
    }

    for (size_t i = 0; i < 3; i++)
    {
      std::string file_name = "/home/tom/Documents/ravi/Recycle_Bin/del/frame_" + utility::to_string(frame) +"_kinect_" + utility::to_string(i) + ".pcd";
      pcl::io::savePCDFileASCII (file_name, *clouds[i]);
    }
    frame++;
    //pcl::io::savePCDFileASCII ("/home/tom/Documents/ravi/Recycle_Bin/del/2.pcd", *clouds[1]);
    //pcl::io::savePCDFileASCII ("/home/tom/Documents/ravi/Recycle_Bin/del/3.pcd", *clouds[2]);

    bool all_success = true;
    pcl::ModelCoefficients spheres_coff[3];
    for (size_t i = 0; i < KINECT_COUNT; i++)
      all_success = all_success && sphere_detector->segmentSphere(clouds[i], spheres_coff[i]);

    if (all_success)
    {
      ROS_INFO_STREAM("Sphere detection successfull");

      recordBallPositionWrtBaxter(ee_msg);
      recordBallPositionWrtKinect(spheres_coff);

      for (size_t i = 0; i < KINECT_COUNT; i++)
      {
          std::string cloud_id = "cloud" + utility::to_string(i);
          if (!pc_viewers.at(i)->updatePointCloud(clouds[i], cloud_id))
              pc_viewers.at(i)->addPointCloud(clouds[i], cloud_id);


          std::string sphere_id = "detected_sphere" + utility::to_string(i);
          pcl::PointXYZ detected_sphere(spheres_coff[i].values[0], spheres_coff[i].values[1], spheres_coff[i].values[2]);
          if (!pc_viewers.at(i)->updateSphere(detected_sphere, spheres_coff[i].values[3], 0.2, 1.0, 0.3, sphere_id))
              pc_viewers.at(i)->addSphere(detected_sphere, spheres_coff[i].values[3], 0.2, 1.0, 0.3, sphere_id);

          // force the viewer to show updaded cloud
          pc_viewers.at(i)->setCameraParameters(camera);
          pc_viewers.at(i)->spinOnce();
      }
    }
    else
    {
      ROS_WARN_STREAM("Sphere detection failed");
    }

    still_processing.data = false; // we have done the processing
    data_collection_progress_pub.publish(still_processing);
}

void DataCollectorMulti::initPCViewers(std::string cam_file)
{
    // allocate memory for trajectory of all kinects
    position_wrt_kinects.resize(KINECT_COUNT);

    std::vector<std::string> cam_param;
    bool result = utility::loadCameraParametersPCL(cam_file, cam_param);
    result = result && utility::getCameraParametersPCL(cam_param, camera);
    ROS_DEBUG_STREAM("loadCameraParametersPCL returned " << result);

    for (size_t i = 0; i < KINECT_COUNT; i++)
    {
        std::string visualizer_name = "Point Cloud " + utility::to_string(i + 1);
        boost::shared_ptr<pcl::visualization::PCLVisualizer> pc_viewer(new pcl::visualization::PCLVisualizer(visualizer_name));
        //pcl::visualization::Camera new_cam;

        pc_viewer->initCameraParameters();
        pc_viewer->setCameraParameters(camera);
        //kinect_viewer->spinOnce();

        // store it
        pc_viewers.push_back(pc_viewer);
    }
}

void DataCollectorMulti::init(ros::NodeHandle nh)
{
    float radius, offset;
    int k_neighbors, max_itr;
    std::string min_hsv, max_hsv, cam_file;
    double weight, d_thresh, prob, tolerance, epsilon;

    nh.getParam("min_hsv", min_hsv);
    nh.getParam("max_hsv", max_hsv);
    nh.getParam("radius", radius);
    nh.getParam("offset", offset);

    // parameters for SAC_RANSAC
    nh.getParam("k_neighbors", k_neighbors);
    nh.getParam("max_itr", max_itr);
    nh.getParam("weight", weight);
    nh.getParam("d_thresh", d_thresh);
    nh.getParam("prob", prob);
    nh.getParam("tolerance", tolerance);
    nh.getParam("epsilon", epsilon);

    nh.getParam("cam_file", cam_file);

    for (size_t i = 1; i <= KINECT_COUNT; i++)
    {
      std::string pc_topic;
      std::string pc_topic_name = "pc" + utility::to_string(i) + "_topic";
      nh.getParam(pc_topic_name, pc_topic);
      pc_topics.push_back(pc_topic);
    }

    nh.getParam("ee_topic", ee_topic);

    nh.getParam("data_dir", data_dir);

    std::vector<float> min_hsv_values = utility::stringToArray(min_hsv);
    std::vector<float> max_hsv_values = utility::stringToArray(max_hsv);

    // initialize sphere detector
    pcl_project::RansacParams ransac_params(k_neighbors, max_itr, weight, d_thresh, prob, tolerance, epsilon);
    sphere_detector = new pcl_project::SphereDetector(radius, &min_hsv_values, &max_hsv_values, &ransac_params);

    // define homogeneous transformation matrix for sphere
    t_ball_wrt_ee.setIdentity(); // considering no rotation
    t_ball_wrt_ee(2, 3) = radius + offset; // distance (m) of the ball center from ee

    baxter_arm_motion_state = 0; //moving:0, stop:1, finished:2
    still_processing.data = false;

    // initialize point cloud viewers
    initPCViewers(cam_file);
}

DataCollectorMulti::DataCollectorMulti()
{
  frame=0;
    ros::NodeHandle nh("~");
    init(nh);

    message_filters::Subscriber<baxter_core_msgs::EndpointState> baxter_arm_sub(nh, ee_topic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud1_sub(nh, pc_topics.at(0), 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud2_sub(nh, pc_topics.at(1), 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud3_sub(nh, pc_topics.at(2), 1);

    typedef message_filters::sync_policies::ApproximateTime<baxter_core_msgs::EndpointState, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;

    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), baxter_arm_sub, point_cloud1_sub, point_cloud2_sub, point_cloud3_sub);
    sync.registerCallback(boost::bind(&DataCollectorMulti::callback, this, _1, _2, _3, _4));

    std::string data_collection_progress_topic = "/multiple_kinect_baxter_calibration/is_data_collection_happening";
    std::string baxter_arm_motion_status_topic = "/multiple_kinect_baxter_calibration/baxter_arm_motion_status";

    data_collection_progress_pub = nh.advertise<std_msgs::Bool>(data_collection_progress_topic, 10);
    baxter_arm_motion_status_sub = nh.subscribe(baxter_arm_motion_status_topic, 10, &DataCollectorMulti::baxterArmMotionStatusCallback, this);
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_collector_multi_node", ros::init_options::AnonymousName);
    DataCollectorMulti dc;
    return 0;
}
