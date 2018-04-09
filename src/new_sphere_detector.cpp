/**
* sphere_detector.cpp: class file for sphere_detector
* Author: Ravi Joshi
* Date: 2018/02/20
*/

#include <utility.h>
#include <stdlib.h>

//#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/angles.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <sensor_msgs/Image.h>
#include <pcl/io/pcd_io.h>

#define outside_limit(num, min, max) (num < min || num > max)

int main(int argc, char** argv)
{
    pcl::visualization::PCLVisualizer pc_viewer("Point Cloud Viewer");
    pcl::visualization::PCLVisualizer seg_viewer("Segmentated Cloud Viewer");

    std::string pcdFileName = "/home/tom/Documents/ravi/Recycle_Bin/no-light/frame_1_kinect_1.pcd";

    std::string cam_file = "/home/tom/ros_ws/src/multiple_kinect_baxter_calibration/files/libfreenect.cam";
    pcl::visualization::Camera camera;
    std::vector<std::string> cam_param;
    utility::loadCameraParametersPCL(cam_file, cam_param);
    utility::getCameraParametersPCL(cam_param, camera);



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p3 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_body (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile(pcdFileName, *cloud);

    float x_min = -0.5, y_min = -0.1, z_min = +0.7;
    float x_max =  0.0, y_max =  0.4, z_max = +4.0;


    size_t count = 0;
    cloud_filtered->header = cloud->header;
    cloud_filtered->height = 1;
    cloud_filtered->points.resize(cloud->points.size());

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        if (outside_limit(cloud->points[i].x, x_min, x_max) ||
            outside_limit(cloud->points[i].y, y_min, y_max) ||
            outside_limit(cloud->points[i].z, z_min, z_max))
            continue;

        cloud_filtered->points[count].x = cloud->points[i].x;
        cloud_filtered->points[count].y = cloud->points[i].y;
        cloud_filtered->points[count].z = cloud->points[i].z;
        cloud_filtered->points[count].r = cloud->points[i].r;
        cloud_filtered->points[count].g = cloud->points[i].g;
        cloud_filtered->points[count].b = cloud->points[i].b;

        count++;
    }

    // Resize to the correct size
    if (count != cloud->points.size())
        cloud_filtered->points.resize(count);

    cloud_filtered->width = count;

    /*
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> x_pass;
    x_pass.setInputCloud (cloud);
    x_pass.setFilterFieldName ("x");
    x_pass.setFilterLimits (x_min, x_max);
    x_pass.filter (*p1);

    pcl::PassThrough<pcl::PointXYZRGB> y_pass;
    y_pass.setInputCloud (p1);
    y_pass.setFilterFieldName ("y");
    y_pass.setFilterLimits (y_min, y_max);
    y_pass.filter (*p2);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (p2);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_min, z_max);
    pass.filter (*filtered_body);
    */



    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);
    ne.setKSearch(10);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
    seg.setNormalDistanceWeight(0.05);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.005);
    seg.setProbability(0.99999);
    seg.setRadiusLimits(0.034 - 0.02, 0.034 + 0.02);
    seg.setEpsAngle(pcl::deg2rad(15.0));

    /*
    sensor_msgs::Image image;
    pcl::toROSMsg (*cloud_filtered, image); //convert the cloud to image

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
      std::cout << "cv_bridge exception: " << e.what() << std::endl;
      return -1;
    }

    cv::Mat cv_img =  cv_ptr->image;
    cv::imwrite( "/home/tom/Documents/ravi/Recycle_Bin/img/image.png", cv_img);
    */

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    rgb_cloud = cloud_filtered;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients coefficients;

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(rgb_cloud);
    ne.compute(*cloud_normals);
    seg.setInputCloud(rgb_cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, coefficients);

    bool status = inliers->indices.size() > 0;




    //pcl::PointXYZ detectedSphere(coefficients.values[0], coefficients.values[1], coefficients.values[2]);

    pc_viewer.addPointCloud(cloud, "raw_cloud");
    //pc_viewer.addSphere(detectedSphere, coefficients.values[3], 0, 255, 0, "detected_sphere");
    pc_viewer.initCameraParameters();
    pc_viewer.addCoordinateSystem (1.0);
    pc_viewer.setCameraParameters(camera);

    seg_viewer.addPointCloud(cloud_filtered, "seg_cloud");
    seg_viewer.initCameraParameters();
    //seg_viewer.addCoordinateSystem (1.0);
    seg_viewer.setCameraParameters(camera);

    seg_viewer.spin();

    /*
    while(1)
    {
        pc_viewer.spinOnce();
        seg_viewer.spinOnce();
        sleep(10);
    }
    */


    return 0;
}
