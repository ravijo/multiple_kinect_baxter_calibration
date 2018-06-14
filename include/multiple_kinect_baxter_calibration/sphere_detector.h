/**
 * sphere_detector.h: header file for sphere_detector
 * Author: Ravi Joshi
 * Date: 2018/02/09
 */

#ifndef MULTIPLE_KINECT_BAXTER_CALIBRATION_SPHERE_DETECTOR_H_
#define MULTIPLE_KINECT_BAXTER_CALIBRATION_SPHERE_DETECTOR_H_

#include <vector>

// pcl headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// vtk header
#include <vtkRenderer.h>
#include <vtkPointPicker.h>
#include <vtkRenderWindowInteractor.h>

namespace pcl_utility
{
class RansacParams
{
public:
    int k_neighbors, max_itr;
    double weight, d_thresh, prob, tolerance, epsilon;

    RansacParams(int k_neighbors_ransac = 10, int max_itr_ransac = 1000,
        double weight_ransac = 0.05, double d_thresh_ransac = 0.005, double prob_ransac = 0.99999,
        double tolerance_ransac = 0.02, double epsilon_ransac = 15);
};

RansacParams::RansacParams(int k_neighbors_ransac, int max_itr_ransac, double weight_ransac,
    double d_thresh_ransac, double prob_ransac, double tolerance_ransac, double epsilon_ransac)
    : k_neighbors(k_neighbors_ransac)
    , max_itr(max_itr_ransac)
    , weight(weight_ransac)
    , d_thresh(d_thresh_ransac)
    , prob(prob_ransac)
    , tolerance(tolerance_ransac)
    , epsilon(epsilon_ransac)
{
}

class SphereDetector
{
public:
    /*
     * Constructor of SphereDetector class
     * Input:
     *   float sphere_radius = radius of the sphere in m
     *   std::vector<int>* min_hsv = minimum HSV values
     *   std::vector<int>* max_hsv = maximum HSV values
     *   RansacParams* ransac_params = Ransac params
     *   int overflow_offset = pixels for considering sphere outside window
     *                         default value is 10
     */
    SphereDetector(float sphere_radius, std::vector<int>* min_hsv, std::vector<int>* max_hsv,
        RansacParams* ransac_params, int overflow_offset = 10);

    /*
     * This function segments the sphere from point cloud
     * Input:
     *   pcl::visualization::PCLVisualizer* viewer = Pointer to PCLVisualizer
     *                                               for grabbing image of
     *                                               the rendered point cloud
     *   pcl::PointCloud<pcl::PointXYZRGB>& raw_cloud = point cloud for
     *                                                  sphere detection
     * Output:
     *   pcl::ModelCoefficients& coefficients = coefficients for detected sphere
     * Returns:
     *   true if sphere is identified, false otherwise
     */
    bool segmentSphere(pcl::visualization::PCLVisualizer* viewer,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud,
        pcl::ModelCoefficients& coefficients);

private:
    void initSphereDetector();

    bool validateDetection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    void get2DPoints(std::vector<cv::Point>& points);

    bool getBoundingRect(cv::Mat image, cv::Rect& bound_rect);

    void crop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out,
        Eigen::Vector4f& min, Eigen::Vector4f& max);

    bool convertMouseCoordsTo3DPC(vtkRenderWindowInteractor* iren, vtkPointPicker* point_picker,
        int mouse_x, int mouse_y, Eigen::Vector3f& point);

    void get3DMinMaxRange(pcl::visualization::PCLVisualizer* viewer, cv::Size img, cv::Rect rect,
        Eigen::Vector4f& min_3d, Eigen::Vector4f& max_3d);

    // src: https://stackoverflow.com/a/6990800
    typedef cv::Scalar* ScalarPtr;
    ScalarPtr min_hsv_ptr, max_hsv_ptr;

    float radius;
    int k_neighbors, max_itr, offset;
    double weight, d_thresh, prob, tolerance, epsilon;
    pcl::PointXY sphere_z_min_max;
    pcl::CropBox<pcl::PointXYZRGB> box_filter;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
};
}

#endif /* MULTIPLE_KINECT_BAXTER_CALIBRATION_SPHERE_DETECTOR_H_ */
