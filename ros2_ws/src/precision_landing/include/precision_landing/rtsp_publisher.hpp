#ifndef PRECISION_LANDING__RTSP_PUBLISHER_HPP_
#define PRECISION_LANDING__RTSP_PUBLISHER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace precision_landing
{

class RtspPublisher : public rclcpp::Node
{
public:
  RtspPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~RtspPublisher();

private:
  bool open_capture();
  sensor_msgs::msg::CameraInfo build_camera_info();
  void timer_callback();

  // Parameters
  std::string rtsp_url_;
  std::string frame_id_;
  bool flip_180_;
  double target_fps_;
  int image_width_;
  int image_height_;
  double camera_fx_;
  double camera_fy_;
  double camera_cx_;
  double camera_cy_;
  std::vector<double> camera_d_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

  // OpenCV Capture and CvBridge
  cv::VideoCapture cap_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Stats
  int frame_count_{0};
  int fail_count_{0};
  const int max_consecutive_fails_{30};
};

}  // namespace precision_landing

#endif  // PRECISION_LANDING__RTSP_PUBLISHER_HPP_
