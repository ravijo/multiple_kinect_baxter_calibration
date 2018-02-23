/**
* data_collector.cpp: class file for collecting data
* Author: Ravi Joshi
* Date: 2018/02/17
*/

#include <vector>
#include <ros/ros.h>
#include <csvfile.h>
#include <Eigen/Geometry>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <sphere_detector.h>
#include <sensor_msgs/Image.h>
#include <pcl/filters/filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <baxter_core_msgs/EndpointState.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// baxter arm motion state (moving:0, stop:1, finished:2)
#define MOVING 0
#define STOP 1
#define FINISHED 2

class DataCollector
{
  private:
    int baxter_arm_motion_state; // moving:0, stop:1, finished:2
    Eigen::Matrix4d t_ball_wrt_ee;
    std_msgs::Bool still_processing;
    std::string pc_topic, ee_topic, data_dir;
    ros::Subscriber baxter_arm_motion_status_sub;
    pcl_project::SphereDetector* sphere_detector;
    pcl::visualization::PCLVisualizer* pc_viewer;
    ros::Publisher data_collection_progress_pub;
    std::vector<std::vector<float> > position_wrt_baxter;
    std::vector<std::vector<float> > position_wrt_kinect;

    void saveData();
    void init(ros::NodeHandle nh);
    std::vector<float> stringToArray(std::string str);
    bool loadCameraParametersPCL(const std::string &file);
    bool getCameraParametersPCL(const std::vector<std::string> &camera);
    void baxterArmMotionStatusCallback(const std_msgs::Int8::ConstPtr& msg);
    void recordBallPositionWrtKinect(pcl::ModelCoefficients& sphere_coefficients);
    void recordBallPositionWrtBaxter(baxter_core_msgs::EndpointStateConstPtr ee_msg);
    void writeCSV(const std::string file_name, const std::string header, std::vector<std::vector<float> > data);
    void getPointCloudFromMsg(sensor_msgs::PointCloud2ConstPtr msg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void callback(const baxter_core_msgs::EndpointStateConstPtr& ee_msg, const sensor_msgs::PointCloud2ConstPtr& pc_msg);
    inline void PointCloudXYZRGBAtoXYZRGB(pcl::PointCloud<pcl::PointXYZRGBA>& in, pcl::PointCloud<pcl::PointXYZRGB>& out);
  public:
    DataCollector();
};

//src: https://github.com/PointCloudLibrary/pcl/blob/master/visualization/src/interactor_style.cpp
bool DataCollector::loadCameraParametersPCL(const std::string &file)
{
  std::ifstream fs;
  std::string line;
  std::vector<std::string> camera;
  bool ret;

  fs.open (file.c_str());
  if (!fs.is_open ())
    return false;

  while (!fs.eof ())
  {
    getline (fs, line);
    if (line == "")
      continue;

    boost::split (camera, line, boost::is_any_of ("/"), boost::token_compress_on);
    break;
  }
  fs.close ();

  return getCameraParametersPCL (camera);
}

bool DataCollector::getCameraParametersPCL(const std::vector<std::string> &camera)
{
  pcl::visualization::Camera camera_temp;

  // look for '/' as a separator
  if (camera.size () != 7)
  {
    pcl::console::print_error("[PCLVisualizer::getCameraParameters] Camera parameters given, but with an invalid number of options (%lu vs 7)!\n", static_cast<unsigned long> (camera.size ()));
    return false;
  }

  std::string clip_str  = camera.at(0);
  std::string focal_str = camera.at(1);
  std::string pos_str   = camera.at(2);
  std::string view_str  = camera.at(3);
  std::string fovy_str  = camera.at(4);
  std::string win_size_str = camera.at(5);
  std::string win_pos_str  = camera.at(6);

  // Get each camera setting separately and parse for ','
  std::vector<std::string> clip_st;
  boost::split (clip_st, clip_str, boost::is_any_of (","), boost::token_compress_on);
  if (clip_st.size () != 2)
  {
    pcl::console::print_error("[PCLVisualizer::getCameraParameters] Invalid parameters given for camera clipping angle!\n");
    return false;
  }

  camera_temp.clip[0] = atof(clip_st.at(0).c_str());
  camera_temp.clip[1] = atof(clip_st.at(1).c_str());

  std::vector<std::string> focal_st;
  boost::split (focal_st, focal_str, boost::is_any_of (","), boost::token_compress_on);
  if (focal_st.size () != 3)
  {
    pcl::console::print_error("[PCLVisualizer::getCameraParameters] Invalid parameters given for camera focal point!\n");
    return false;
  }

  camera_temp.focal[0] = atof(focal_st.at(0).c_str());
  camera_temp.focal[1] = atof(focal_st.at(1).c_str());
  camera_temp.focal[2] = atof(focal_st.at(2).c_str());

  std::vector<std::string> pos_st;
  boost::split (pos_st, pos_str, boost::is_any_of (","), boost::token_compress_on);
  if (pos_st.size () != 3)
  {
    pcl::console::print_error("[PCLVisualizer::getCameraParameters] Invalid parameters given for camera position!\n");
    return false;
  }

  camera_temp.pos[0] = atof(pos_st.at(0).c_str());
  camera_temp.pos[1] = atof(pos_st.at(1).c_str());
  camera_temp.pos[2] = atof(pos_st.at(2).c_str());

  std::vector<std::string> view_st;
  boost::split (view_st, view_str, boost::is_any_of (","), boost::token_compress_on);
  if (view_st.size () != 3)
  {
    pcl::console::print_error("[PCLVisualizer::getCameraParameters] Invalid parameters given for camera viewup!\n");
    return false;
  }

  camera_temp.view[0] = atof(view_st.at(0).c_str());
  camera_temp.view[1] = atof(view_st.at(1).c_str());
  camera_temp.view[2] = atof(view_st.at(2).c_str());

  std::vector<std::string> fovy_size_st;
  boost::split (fovy_size_st, fovy_str, boost::is_any_of (","), boost::token_compress_on);
  if (fovy_size_st.size () != 1)
  {
    pcl::console::print_error("[PCLVisualizer::getCameraParameters] Invalid parameters given for field of view angle!\n");
    return false;
  }

  camera_temp.fovy = atof(fovy_size_st.at(0).c_str());

  std::vector<std::string> win_size_st;
  boost::split (win_size_st, win_size_str, boost::is_any_of (","), boost::token_compress_on);
  if (win_size_st.size () != 2)
  {
    pcl::console::print_error("[PCLVisualizer::getCameraParameters] Invalid parameters given for window size!\n");
    return false;
  }

  camera_temp.window_size[0] = atof(win_size_st.at(0).c_str());
  camera_temp.window_size[1] = atof(win_size_st.at(1).c_str());

  std::vector<std::string> win_pos_st;
  boost::split (win_pos_st, win_pos_str, boost::is_any_of (","), boost::token_compress_on);
  if (win_pos_st.size () != 2)
  {
    pcl::console::print_error("[PCLVisualizer::getCameraParameters] Invalid parameters given for window position!\n");
    return false;
  }

  camera_temp.window_pos[0] = atof(win_pos_st.at(0).c_str());
  camera_temp.window_pos[1] = atof(win_pos_st.at(1).c_str());

  pc_viewer->setCameraParameters(camera_temp);
  return true;
}

void DataCollector::recordBallPositionWrtBaxter(baxter_core_msgs::EndpointStateConstPtr ee_msg)
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

void DataCollector::recordBallPositionWrtKinect(pcl::ModelCoefficients& sphere_coefficients)
{
    std::vector<float> kinect;
    for (int i = 0; i < 3; i++)
        kinect.push_back(sphere_coefficients.values[i]);

    position_wrt_kinect.push_back(kinect);
}

void DataCollector::baxterArmMotionStatusCallback(const std_msgs::Int8::ConstPtr& msg)
{
    baxter_arm_motion_state = msg->data;
}

void DataCollector::getPointCloudFromMsg(sensor_msgs::PointCloud2ConstPtr msg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    std::vector<int> indices;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZRGBA> temp_cloud;

    /*
    * It was found that even though point cloud ros message field says that libfreenect2
    * point cloud is 'rgb', it is 'rgba'. Hence the code below assumes that incoming
    * point cloud is 'rgba' and it converts it to 'rgb' to further use
    */
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, temp_cloud);
    PointCloudXYZRGBAtoXYZRGB(temp_cloud, *cloud);

    /*
    * point cloud received from libfreenect2 shows that it is dense point cloud
    * which means it shouldn't contain any 'nan' but 'nan' was found. Hence in
    * order to remove 'nan', we first need to make it non-dense. we should
    * make it dense, once 'nan' are removed.
    */
    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    cloud->is_dense = true;
}

inline void DataCollector::PointCloudXYZRGBAtoXYZRGB(pcl::PointCloud<pcl::PointXYZRGBA>& in, pcl::PointCloud<pcl::PointXYZRGB>& out)
{
  out.width   = in.width;
  out.height  = in.height;
  out.points.resize(in.points.size());
  for (size_t i = 0; i < in.points.size (); i++)
  {
    out.points[i].x = in.points[i].x;
    out.points[i].y = in.points[i].y;
    out.points[i].z = in.points[i].z;
    out.points[i].r = in.points[i].r;
    out.points[i].g = in.points[i].g;
    out.points[i].b = in.points[i].b;
  }
}

void DataCollector::saveData()
{
    std::string baxter = data_dir + "/position_wrt_baxter.csv";
    std::string kinect = data_dir + "/position_wrt_kinect.csv";
    std::string header = "position_x,position_y,position_z";

    ROS_INFO_STREAM("Saving collected data in following files: " << baxter << " and " << kinect);

    writeCSV(baxter, header, position_wrt_baxter);
    writeCSV(kinect, header, position_wrt_kinect);
}

void DataCollector::callback(const baxter_core_msgs::EndpointStateConstPtr& ee_msg, const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    getPointCloudFromMsg(pc_msg, cloud);

    // show caputed point cloud
    if (!pc_viewer->updatePointCloud(cloud, "cloud"))
        pc_viewer->addPointCloud(cloud, "cloud");

    // exit and save the data
    if (baxter_arm_motion_state == FINISHED)
    {
      if(position_wrt_kinect.empty())
        ROS_ERROR_STREAM("Couldn't record any data. Run the program again.");
      else
        saveData();

      ROS_INFO_STREAM("Exiting now...");
      ros::shutdown();
      return;
    }

    // don't proceed further if robot is moving or if we are processing the existing data
    if (still_processing.data || (baxter_arm_motion_state == MOVING)) return;

    still_processing.data = true; // we are about to start processing
    data_collection_progress_pub.publish(still_processing);

    pcl::ModelCoefficients sphere_coff;
    bool success = sphere_detector->segmentSphere(cloud, sphere_coff);

    if (success)
    {
        ROS_DEBUG("Sphere detection successfull");

        recordBallPositionWrtBaxter(ee_msg);
        recordBallPositionWrtKinect(sphere_coff);

        pcl::PointXYZ detected_sphere(sphere_coff.values[0], sphere_coff.values[1], sphere_coff.values[2]);
        if (!pc_viewer->updateSphere(detected_sphere, sphere_coff.values[3], 0.2, 1.0, 0.3, "detected_sphere"))
            pc_viewer->addSphere(detected_sphere, sphere_coff.values[3], 0.2, 1.0, 0.3, "detected_sphere");

        pc_viewer->spinOnce();
    }
    else
    {
        ROS_WARN("Sphere detection failed");
    }

    still_processing.data = false; // we have done the processing
    data_collection_progress_pub.publish(still_processing);
}

void DataCollector::writeCSV(std::string file_name, std::string header, std::vector<std::vector<float> > data)
{
    try
    {
        csvfile csv(file_name);
        csv << header << endrow;

        for (int i = 0; i < data.size(); i++)
        {
            for (int j = 0; j < data[i].size(); j++)
                csv << data[i][j];
            csv << endrow;
        }
        csv.close();
    }
    catch (std::exception& ex)
    {
        ROS_ERROR_STREAM("Unable to save tracking data to CSV file. " << ex.what());
    }
}

void DataCollector::init(ros::NodeHandle nh)
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

    nh.getParam("pc_topic", pc_topic);
    nh.getParam("ee_topic", ee_topic);

    nh.getParam("data_dir", data_dir);

    std::vector<float> min_hsv_values = stringToArray(min_hsv);
    std::vector<float> max_hsv_values = stringToArray(max_hsv);

    // initialize sphere detector
    pcl_project::RansacParams ransac_params(k_neighbors, max_itr, weight, d_thresh, prob, tolerance, epsilon);
    sphere_detector = new pcl_project::SphereDetector(radius, &min_hsv_values, &max_hsv_values, &ransac_params);

    // define homogeneous transformation matrix for sphere
    t_ball_wrt_ee.setIdentity(); // considering no rotation
    t_ball_wrt_ee(2, 3) = radius + offset; // distance (m) of the ball center from ee

    baxter_arm_motion_state = 0; //moving:0, stop:1, finished:2
    still_processing.data = false;

    // initialize point cloud viewer
    pc_viewer = new pcl::visualization::PCLVisualizer("Cloud Viewer");
    pc_viewer->initCameraParameters();
    bool result = loadCameraParametersPCL(cam_file);
    ROS_DEBUG_STREAM("loadCameraParametersPCL returned " << result);
}

std::vector<float> DataCollector::stringToArray(std::string str)
{
    // remove spaces
    str.erase(std::remove_if(str.begin(), str.end(), ::isspace), str.end());

    // split string by comma
    std::vector<std::string> str_array;
    boost::split(str_array, str, boost::is_any_of(","));

    // remove opening and closing bracket
    str_array[0] = str_array[0].substr(1);
    str_array[2] = str_array[2].substr(0, str_array[2].size() -1);

    // convert all string to float
    std::vector<float> float_array(str_array.size());
    for (int i = 0; i < str_array.size(); i++)
        float_array[i] = atof(str_array[i].c_str());

    return float_array;
}

DataCollector::DataCollector()
{
    ros::NodeHandle nh("~");
    init(nh);

    message_filters::Subscriber<baxter_core_msgs::EndpointState> baxter_arm_sub(nh, ee_topic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub(nh, pc_topic, 1);

    typedef message_filters::sync_policies::ApproximateTime<baxter_core_msgs::EndpointState, sensor_msgs::PointCloud2> SyncPolicy;

    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), baxter_arm_sub, point_cloud_sub);
    sync.registerCallback(boost::bind(&DataCollector::callback, this, _1, _2));

    std::string data_collection_progress_topic = "/multiple_kinect_baxter_calibration/is_data_collection_happening";
    std::string baxter_arm_motion_status_topic = "/multiple_kinect_baxter_calibration/baxter_arm_motion_status";

    data_collection_progress_pub = nh.advertise<std_msgs::Bool>(data_collection_progress_topic, 10);
    baxter_arm_motion_status_sub = nh.subscribe(baxter_arm_motion_status_topic, 10, &DataCollector::baxterArmMotionStatusCallback, this);
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_collector_node", ros::init_options::AnonymousName);
    DataCollector dc;
    return 0;
}
