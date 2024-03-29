/**
 * segment_image.cpp: test file for finding HSV range for image segmentation
 * Author: Ravi Joshi
 * Date: 2018/04/18
 * Source:
 * https://docs.opencv.org/2.4/doc/tutorials/highgui/trackbar/trackbar.html
 */

// ros header
#include <ros/ros.h>

// opencv2 header
#include <opencv2/opencv.hpp>

using namespace cv;

// For HSV, hue range is [0, 179], 
// saturation and value range is [0, 255]
// https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html
const int opencv_max_h = 179;
const int opencv_max_sv = 255;

// global variables
int min_h = 0, min_s = 0, min_v = 0;
int max_h = opencv_max_h, max_s = opencv_max_sv, max_v = opencv_max_sv;
cv::Mat src_image;
std::string window_text = "Find HSV range (Press 'q' to exit)";

inline void segment_image()
{
  cv::Mat image = src_image.clone();
  if (image.channels() == 4)
      cv::cvtColor(image, image, cv::COLOR_BGRA2BGR, 3);  // remove alpha channel

  cv::GaussianBlur(image, image, cv::Size(5, 5), 0, 0);
  cv::Mat hsv_image, binary_image;
  cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);  // convert to hsv

  cv::inRange(hsv_image, cv::Scalar(min_h, min_s, min_v),
              cv::Scalar(max_h, max_s, max_v), binary_image);
  cv::imshow(window_text, binary_image);
}

/**
 * @function on_trackbar
 * @brief Callback for trackbar
 */
void on_min_h_trackbar(int, void*)
{
  segment_image();
}

void on_min_s_trackbar(int, void*)
{
  segment_image();
}

void on_min_v_trackbar(int, void*)
{
  segment_image();
}

void on_max_h_trackbar(int, void*)
{
  segment_image();
}

void on_max_s_trackbar(int, void*)
{
  segment_image();
}

void on_max_v_trackbar(int, void*)
{
  segment_image();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "view_image_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");
  std::string image_file;
  nh.getParam("file", image_file);

  if (image_file.empty())
    std::cerr << "Image is not provided. Kindly use the following way-\n"
              << "_file:=/home/ravi/Desktop/image.png" << std::endl;

  cv::namedWindow(window_text, CV_WINDOW_AUTOSIZE);
  src_image = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);
  if (!src_image.data)  // check for invalid input
  {
    std::cerr << "Could not open or find the image '" << image_file << "'"
              << std::endl;
    return -1;
  }
  else
    std::cout << "Loading '" << image_file << "'" << std::endl;

  // create windows
  cv::namedWindow(window_text, CV_WINDOW_AUTOSIZE);

  // create trackbars
  createTrackbar("Minimum Hue", window_text, &min_h, opencv_max_h,
                 on_min_h_trackbar);
  createTrackbar("Minimum Saturation", window_text, &min_s, opencv_max_sv,
                 on_min_s_trackbar);
  createTrackbar("Minimum Value", window_text, &min_v, opencv_max_sv,
                 on_min_v_trackbar);
  createTrackbar("Maximum Hue", window_text, &max_h, opencv_max_h,
                 on_max_h_trackbar);
  createTrackbar("Maximum Saturation", window_text, &max_s, opencv_max_sv,
                 on_max_s_trackbar);
  createTrackbar("Maximum Value", window_text, &max_v, opencv_max_sv,
                 on_max_v_trackbar);

  /// show some stuff
  on_min_h_trackbar(min_h, 0);
  on_min_s_trackbar(min_s, 0);
  on_min_v_trackbar(min_v, 0);
  on_max_h_trackbar(max_h, 0);
  on_max_s_trackbar(max_s, 0);
  on_max_v_trackbar(max_v, 0);

  cv::imshow(window_text, src_image);

  /// wait until user press some key
  cv::waitKey(0);
  return 0;
}
