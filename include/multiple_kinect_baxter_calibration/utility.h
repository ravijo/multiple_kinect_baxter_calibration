/**
 * utility.h: various utility functions
 * Author: Ravi Joshi
 * Date: 2018/02/24
 */

#ifndef MULTIPLE_KINECT_BAXTER_CALIBRATION_UTILITY_H_
#define MULTIPLE_KINECT_BAXTER_CALIBRATION_UTILITY_H_

#include <vector>

// header file for writing csv file
#include <csvfile.h>

// PCL headers
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

// opencv2 header
#include <opencv2/opencv.hpp>

// vtk header
#include <vtkRenderWindow.h>

// preprocessor returns true if number is outside given limit
#define outside_limit(num, min, max) (num < min || num > max)

namespace utility {
// due to PCL issues we are not able to use C++ 11 std. Let's define our own functions
// src: https://stackoverflow.com/a/20861692
template<typename T> std::string to_string(const T& n) {
	std::ostringstream stm;
	stm << n;
	return stm.str();
}

inline void PointCloudXYZRGBtoXYZHSV(pcl::PointCloud<pcl::PointXYZRGB>& in,
		pcl::PointCloud<pcl::PointXYZHSV>& out) {
	out.width = in.width;
	out.height = in.height;
	out.points.resize(in.points.size());

	for (size_t i = 0; i < in.points.size(); i++) {
		pcl::PointXYZRGBtoXYZHSV(in.points[i], out.points[i]);
		// fox for a bug in PCL 1.7
		out.points[i].x = in.points[i].x;
		out.points[i].y = in.points[i].y;
		out.points[i].z = in.points[i].z;
	}
}

inline void PointCloudXYZHSVtoXYZRGB(pcl::PointCloud<pcl::PointXYZHSV>& in,
		pcl::PointCloud<pcl::PointXYZRGB>& out) {
	out.width = in.width;
	out.height = in.height;
	out.points.resize(in.points.size());

	for (size_t i = 0; i < in.points.size(); i++)
		pcl::PointXYZHSVtoXYZRGB(in.points[i], out.points[i]);
}

inline void removeNanAndConvertPointCloudToRGB(
		pcl::PointCloud<pcl::PointXYZRGBA>& in,
		pcl::PointCloud<pcl::PointXYZRGB>& out, float min_z, float max_z) {
	size_t count = 0;
	out.header = in.header;
	out.height = 1;
	out.points.resize(in.points.size());

	for (size_t i = 0; i < in.points.size(); i++) {
		if (!pcl_isfinite(in.points[i].x) ||
		!pcl_isfinite(in.points[i].y) ||
		outside_limit(in.points[i].z, min_z, max_z))
			continue;

		out.points[count].x = in.points[i].x;
		out.points[count].y = in.points[i].y;
		out.points[count].z = in.points[i].z;
		out.points[count].r = in.points[i].r;
		out.points[count].g = in.points[i].g;
		out.points[count].b = in.points[i].b;

		count++;
	}

	// Resize to the correct size
	if (count != in.points.size())
		out.points.resize(count);

	out.width = count;
}

// retrieve rendered image by pcl::visualization::PCLVisualizer
cv::Mat getImageFromPCLViewer(pcl::visualization::PCLVisualizer* viewer) {
	vtkSmartPointer < vtkRenderWindow > render = viewer->getRenderWindow();

	int rows = render->GetSize()[1];
	int cols = render->GetSize()[0];
	unsigned char* pixels = render->GetRGBACharPixelData(0, 0, cols - 1,
			rows - 1, 1);

	// compose cv::Mat
	cv::Mat image(rows, cols, CV_8UC4, pixels);
	cv::cvtColor(image, image, cv::COLOR_RGBA2BGRA);
	cv::flip(image, image, 0);

	return image;
}

void getPointCloudFromMsg(sensor_msgs::PointCloud2ConstPtr msg,
		pcl::PointCloud<pcl::PointXYZRGB> &cloud, float min_z = 0.5,
		float max_z = 5.0) {
	pcl::PCLPointCloud2 pcl_pc2;
	pcl::PointCloud < pcl::PointXYZRGBA > temp_cloud;

	/*
	 * It was found that even though point cloud ros message field says that 
	 * libfreenect2 point cloud is 'rgb', it is 'rgba'. Hence the code below 
	 * assumes that incoming point cloud is 'rgba' and it converts it to 'rgb' 
	 * to further use
	 */
	pcl_conversions::toPCL(*msg, pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2, temp_cloud);
	removeNanAndConvertPointCloudToRGB(temp_cloud, cloud, min_z, max_z);
}

bool writeCSV(std::string file_name, std::string header,
		std::vector<std::vector<float> > data, std::string &error) {
	try {
		csvfile csv(file_name);
		csv << header << endrow;

		for (int i = 0; i < data.size(); i++) {
			for (int j = 0; j < data[i].size(); j++)
				csv << data[i][j];
			csv << endrow;
		}
		csv.close();
		return true;
	} catch (std::exception& ex) {
		error = ex.what();
		return false;
	}
}

std::vector<int> stringToArray(std::string str) {
	// remove spaces
	str.erase(std::remove_if(str.begin(), str.end(), ::isspace), str.end());

	// split string by comma
	std::vector<std::string> str_array;
	boost::split(str_array, str, boost::is_any_of(","));

	// remove opening and closing bracket
	str_array[0] = str_array[0].substr(1);
	str_array[2] = str_array[2].substr(0, str_array[2].size() - 1);

	// convert all string to integer
	std::vector<int> num_array(str_array.size());
	for (int i = 0; i < str_array.size(); i++)
		num_array[i] = atoi(str_array[i].c_str());

	return num_array;
}

//src: https://github.com/PointCloudLibrary/pcl/blob/master/visualization/src/interactor_style.cpp
bool loadCameraParametersPCL(const std::string &file,
		std::vector<std::string> &camera) {
	std::ifstream fs;
	std::string line;
	bool ret;

	fs.open(file.c_str());
	if (!fs.is_open())
		return false;

	while (!fs.eof()) {
		getline(fs, line);
		if (line == "")
			continue;

		boost::split(camera, line, boost::is_any_of("/"),
				boost::token_compress_on);
		break;
	}
	fs.close();

	return true;
}

bool getCameraParametersPCL(const std::vector<std::string> &camera,
		pcl::visualization::Camera &new_camera) {
	// look for '/' as a separator
	if (camera.size() != 7) {
		pcl::console::print_error(
				"[PCLVisualizer::getCameraParameters] Camera parameters "
				"given, but with an invalid number of options (%lu vs 7)!\n",
				static_cast<unsigned long>(camera.size()));
		return false;
	}

	std::string clip_str = camera.at(0);
	std::string focal_str = camera.at(1);
	std::string pos_str = camera.at(2);
	std::string view_str = camera.at(3);
	std::string fovy_str = camera.at(4);
	std::string win_size_str = camera.at(5);
	std::string win_pos_str = camera.at(6);

	// Get each camera setting separately and parse for ','
	std::vector<std::string> clip_st;
	boost::split(clip_st, clip_str, boost::is_any_of(","),
			boost::token_compress_on);
	if (clip_st.size() != 2) {
		pcl::console::print_error(
				"[PCLVisualizer::getCameraParameters] Invalid parameters "
				"given for camera clipping angle!\n");
		return false;
	}

	new_camera.clip[0] = atof(clip_st.at(0).c_str());
	new_camera.clip[1] = atof(clip_st.at(1).c_str());

	std::vector<std::string> focal_st;
	boost::split(focal_st, focal_str, boost::is_any_of(","),
			boost::token_compress_on);
	if (focal_st.size() != 3) {
		pcl::console::print_error(
				"[PCLVisualizer::getCameraParameters] Invalid parameters "
				"given for camera focal point!\n");
		return false;
	}

	new_camera.focal[0] = atof(focal_st.at(0).c_str());
	new_camera.focal[1] = atof(focal_st.at(1).c_str());
	new_camera.focal[2] = atof(focal_st.at(2).c_str());

	std::vector<std::string> pos_st;
	boost::split(pos_st, pos_str, boost::is_any_of(","),
			boost::token_compress_on);
	if (pos_st.size() != 3) {
		pcl::console::print_error(
				"[PCLVisualizer::getCameraParameters] Invalid parameters "
				"given for camera position!\n");
		return false;
	}

	new_camera.pos[0] = atof(pos_st.at(0).c_str());
	new_camera.pos[1] = atof(pos_st.at(1).c_str());
	new_camera.pos[2] = atof(pos_st.at(2).c_str());

	std::vector<std::string> view_st;
	boost::split(view_st, view_str, boost::is_any_of(","),
			boost::token_compress_on);
	if (view_st.size() != 3) {
		pcl::console::print_error(
				"[PCLVisualizer::getCameraParameters] Invalid parameters "
				"given for camera viewup!\n");
		return false;
	}

	new_camera.view[0] = atof(view_st.at(0).c_str());
	new_camera.view[1] = atof(view_st.at(1).c_str());
	new_camera.view[2] = atof(view_st.at(2).c_str());

	std::vector<std::string> fovy_size_st;
	boost::split(fovy_size_st, fovy_str, boost::is_any_of(","),
			boost::token_compress_on);
	if (fovy_size_st.size() != 1) {
		pcl::console::print_error(
				"[PCLVisualizer::getCameraParameters] Invalid parameters "
				"given for field of view angle!\n");
		return false;
	}

	new_camera.fovy = atof(fovy_size_st.at(0).c_str());

	std::vector<std::string> win_size_st;
	boost::split(win_size_st, win_size_str, boost::is_any_of(","),
			boost::token_compress_on);
	if (win_size_st.size() != 2) {
		pcl::console::print_error(
				"[PCLVisualizer::getCameraParameters] Invalid parameters "
				"given for window size!\n");
		return false;
	}

	new_camera.window_size[0] = atof(win_size_st.at(0).c_str());
	new_camera.window_size[1] = atof(win_size_st.at(1).c_str());

	std::vector<std::string> win_pos_st;
	boost::split(win_pos_st, win_pos_str, boost::is_any_of(","),
			boost::token_compress_on);
	if (win_pos_st.size() != 2) {
		pcl::console::print_error(
				"[PCLVisualizer::getCameraParameters] Invalid parameters "
				"given for window position!\n");
		return false;
	}

	new_camera.window_pos[0] = atof(win_pos_st.at(0).c_str());
	new_camera.window_pos[1] = atof(win_pos_st.at(1).c_str());

	return true;
}
}
#endif /* MULTIPLE_KINECT_BAXTER_CALIBRATION_UTILITY_H_ */

