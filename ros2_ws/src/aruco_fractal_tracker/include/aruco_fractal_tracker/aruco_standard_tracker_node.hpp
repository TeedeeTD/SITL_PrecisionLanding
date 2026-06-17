#ifndef ARUCO_FRACTAL_TRACKER__ARUCO_STANDARD_TRACKER_NODE_HPP_
#define ARUCO_FRACTAL_TRACKER__ARUCO_STANDARD_TRACKER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace standard_tracker
{
class ArucoStandardTracker : public rclcpp::Node
{
public:
  explicit ArucoStandardTracker(const rclcpp::NodeOptions& options);

private:
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr uav_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lander_state_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::PoseStamped::SharedPtr last_uav_pose_;

  sensor_msgs::msg::CameraInfo last_camera_info_;
  bool camera_info_initialized_{false};

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  std::string dictionary_name_;
  int target_tag_id_{0};
  double marker_size_{0.50};
  std::string camera_frame_{"camera_link"};

  double camera_x_to_east_sign_{1.0};
  double camera_y_to_north_sign_{-1.0};
  double camera_offset_x_{0.1517};
  double camera_offset_y_{0.0};
  bool show_latency_overlay_{true};
  double latency_warn_ms_{100.0};
  size_t frame_count_{0};
  size_t detection_count_{0};
  double last_processing_latency_ms_{0.0};
  double last_source_latency_ms_{0.0};
  bool source_latency_valid_{false};
  rclcpp::Time last_no_detection_log_;
  rclcpp::Time last_pose_log_;
  rclcpp::Time last_pose_failed_log_;
  rclcpp::Time last_latency_log_;
  std::string last_detected_ids_str_{"None"};
  std::string last_lander_state_{"UNKNOWN"};
  double current_fps_{0.0};
  rclcpp::Time last_fps_time_;
  size_t fps_frame_count_{0};

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void drawLatencyOverlay(cv::Mat& image) const;
  void initFallbackIntrinsics();
};
} // namespace standard_tracker

#endif // ARUCO_FRACTAL_TRACKER__ARUCO_STANDARD_TRACKER_NODE_HPP_
