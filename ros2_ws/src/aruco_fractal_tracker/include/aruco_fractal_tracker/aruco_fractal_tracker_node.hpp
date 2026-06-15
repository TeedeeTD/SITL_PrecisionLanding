/*
 * This file is part of the aruco_fractal_tracker distribution (https://github.com/dimianx/aruco_fractal_tracker).
 * Copyright (c) 2024-2025 Dmitry Anikin <dmitry.anikin@proton.me>.
 *
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ARUCO_FRACTAL_TRACKER__ARUCO_FRACTAL_TRACKER_NODE_HPP_
#define ARUCO_FRACTAL_TRACKER__ARUCO_FRACTAL_TRACKER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <aruco/fractaldetector.h>
#include <chrono>
#include <memory>

namespace fractal_tracker
{
class ArucoFractalTracker : public rclcpp::Node
{
public:
  explicit ArucoFractalTracker(const rclcpp::NodeOptions& options);

private:  
  aruco::FractalDetector detector_;

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

  double marker_size_;
  double camera_x_to_east_sign_{-1.0};
  double camera_y_to_north_sign_{1.0};
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
}; // class ArucoFractalTracker
}  // namespace fractal_tracker

#endif  // ARUCO_FRACTAL_TRACKER__ARUCO_FRACTAL_TRACKER_NODE_HPP_
