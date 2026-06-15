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

#include "aruco_fractal_tracker/aruco_fractal_tracker_node.hpp"

#include <stdexcept>
#include <string>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace fractal_tracker
{
ArucoFractalTracker::ArucoFractalTracker(const rclcpp::NodeOptions &options)
  : Node("aruco_fractal_tracker", options)
{
  this->declare_parameter<std::string>("marker_configuration", "");
  this->declare_parameter<double>("marker_size", 0.0);
  this->declare_parameter<bool>("show_latency_overlay", true);
  this->declare_parameter<double>("latency_warn_ms", 100.0);

  auto marker_configuration = this->get_parameter("marker_configuration").get_value<std::string>();
  marker_size_ = this->get_parameter("marker_size").get_value<double>();
  show_latency_overlay_ = this->get_parameter("show_latency_overlay").as_bool();
  latency_warn_ms_ = this->get_parameter("latency_warn_ms").as_double();
  
  detector_.setConfiguration(marker_configuration);

  // Set default/fallback camera parameters (width: 1280, height: 720, HFOV: 1.2 rad)
  // fx = fy = (1280 / 2) / tan(1.2 / 2) = 640 / tan(0.6) = 935.4853
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
    935.4853, 0.0, 640.0,
    0.0, 935.4853, 360.0,
    0.0, 0.0, 1.0);
  cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
  cv::Size image_size(1280, 720);
  aruco::CameraParameters cam_params;
  cam_params.setParams(camera_matrix, dist_coeffs, image_size);
  detector_.setParams(cam_params, marker_size_);

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info_topic", 10, std::bind(&ArucoFractalTracker::cameraInfoCallback, this, std::placeholders::_1));

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_input_topic", 10, std::bind(&ArucoFractalTracker::imageCallback, this, std::placeholders::_1));

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_output_topic", 10);

  marker_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("poses_output_topic", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  last_no_detection_log_ = this->get_clock()->now();
  last_pose_log_ = this->get_clock()->now();
  last_pose_failed_log_ = this->get_clock()->now();
  last_latency_log_ = this->get_clock()->now();
  last_fps_time_ = this->get_clock()->now();

  RCLCPP_INFO(
    this->get_logger(),
    "ArucoFractalTracker ready: marker_configuration=%s marker_size=%.3f "
    "latency_overlay=%s warn=%.1fms",
    marker_configuration.c_str(), marker_size_,
    show_latency_overlay_ ? "on" : "off", latency_warn_ms_);
  RCLCPP_INFO(
    this->get_logger(),
    "Topics before remap: image_input_topic -> image_output_topic, poses_output_topic; "
    "CameraInfo is optional because fallback intrinsics are loaded");
}

void ArucoFractalTracker::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  bool update_needed = false;

  if (!camera_info_initialized_) 
  {
    update_needed = true;
  } 
  else 
  {
    if (msg->width != last_camera_info_.width || msg->height != last_camera_info_.height) 
    {
      update_needed = true;
    } 
    else 
    {
      for (int i = 0; i < 9; ++i) 
      {
        if (std::abs(msg->k[i] - last_camera_info_.k[i]) > 1e-6) 
        {
          update_needed = true;
          break;
        }
      }
      if (!update_needed && msg->d.size() == last_camera_info_.d.size()) 
      {
        for (size_t i = 0; i < msg->d.size(); ++i) 
        {
          if (std::abs(msg->d[i] - last_camera_info_.d[i]) > 1e-6) 
          {
            update_needed = true;
            break;
          }
        }
      } 
      else if (!update_needed) 
      {
        update_needed = true;
      }
    }
  }

  if (!update_needed) 
  {
    return;
  }

  last_camera_info_ = *msg;
  camera_info_initialized_ = true;

  cv::Mat camera_matrix(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i)
  {
    camera_matrix.at<double>(i / 3, i % 3) = msg->k[i];
  }

  cv::Mat dist_coeffs(static_cast<int>(msg->d.size()), 1, CV_64F);
  for (size_t i = 0; i < msg->d.size(); ++i)
  {
    dist_coeffs.at<double>(i, 0) = msg->d[i];
  }

  cv::Size image_size(msg->width, msg->height);

  aruco::CameraParameters cam_params;
  cam_params.setParams(camera_matrix, dist_coeffs, image_size);
  if (!cam_params.isValid())
    throw std::invalid_argument("Invalid camera parameters!");

  detector_.setParams(cam_params, marker_size_);

  RCLCPP_INFO(
    this->get_logger(),
    "CameraInfo updated: size=%ux%u fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
    msg->width, msg->height, msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
}


void ArucoFractalTracker::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  const auto callback_start = std::chrono::steady_clock::now();
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat gray;
  ++frame_count_;
  const auto now = this->get_clock()->now();

  ++fps_frame_count_;
  double elapsed = (now - last_fps_time_).seconds();
  if (elapsed >= 1.0)
  {
    current_fps_ = fps_frame_count_ / elapsed;
    fps_frame_count_ = 0;
    last_fps_time_ = now;
  }

  try 
  {
    cv_ptr = cv_bridge::toCvCopy(msg);
  } 
  catch (cv_bridge::Exception& e) 
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
    return;
  }

  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

  if (detector_.detect(gray))
  {
    detector_.drawMarkers(cv_ptr->image);
    
    std::vector<aruco::Marker> markers = detector_.getMarkers();
    std::string ids_str = "";
    for (size_t i = 0; i < markers.size(); ++i)
    {
      markers[i].draw(cv_ptr->image, cv::Scalar(255, 255, 255), 2);
      if (i > 0) ids_str += ",";
      ids_str += std::to_string(markers[i].id);
    }
    last_detected_ids_str_ = ids_str.empty() ? "None" : ids_str;

    detector_.draw2d(cv_ptr->image);

    if (detector_.poseEstimation())
    {
      cv::Mat tvec = detector_.getTvec();
      cv::Mat rvec = detector_.getRvec();
      detector_.draw3d(cv_ptr->image);
      
      cv::Mat rmatrix;
      cv::Rodrigues(rvec, rmatrix);
      tf2::Matrix3x3 tf2_rot(rmatrix.at<double>(0, 0), rmatrix.at<double>(0, 1), rmatrix.at<double>(0, 2),
                             rmatrix.at<double>(1, 0), rmatrix.at<double>(1, 1), rmatrix.at<double>(1, 2),
                             rmatrix.at<double>(2, 0), rmatrix.at<double>(2, 1), rmatrix.at<double>(2, 2));
        
      tf2::Vector3 tf2_translation(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));
      tf2::Transform tf2_transform(tf2_rot, tf2_translation);
      tf2::Quaternion quat;
      tf2_rot.getRotation(quat);

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = msg->header.frame_id;
      pose.header.stamp = msg->header.stamp;
      pose.pose.position.x = tf2_translation.x();
      pose.pose.position.y = tf2_translation.y();
      pose.pose.position.z = tf2_translation.z();
      pose.pose.orientation.x = quat.getX();
      pose.pose.orientation.y = quat.getY();
      pose.pose.orientation.z = quat.getZ();
      pose.pose.orientation.w = quat.getW();

      marker_pose_pub_->publish(pose);
      ++detection_count_;

      if ((now - last_pose_log_).seconds() >= 1.0)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Fractal marker detected: frames=%zu detections=%zu tvec=[%.2f, %.2f, %.2f] ids=[%s] frame_id=%s",
          frame_count_, detection_count_,
          tf2_translation.x(), tf2_translation.y(), tf2_translation.z(),
          ids_str.c_str(),
          msg->header.frame_id.c_str());
        last_pose_log_ = now;
      }
      
      int base_line = 0;
      int font_face = cv::FONT_HERSHEY_PLAIN;
      double font_scale = 1;
      int thickness = 1;
      int line_height = cv::getTextSize("W", font_face, font_scale, thickness, &base_line).height + 5;

      cv::Point pos_text_pos(10, 10 + line_height);
      cv::putText(cv_ptr->image, "POSITION", pos_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, "POSITION", pos_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      pos_text_pos.y += line_height;
      cv::putText(cv_ptr->image, cv::format("X=%.2f", tf2_translation.x()), pos_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("X=%.2f", tf2_translation.x()), pos_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      pos_text_pos.y += line_height;
      cv::putText(cv_ptr->image, cv::format("Y=%.2f", tf2_translation.y()), pos_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("Y=%.2f", tf2_translation.y()), pos_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      pos_text_pos.y += line_height;
      cv::putText(cv_ptr->image, cv::format("Z=%.2f", tf2_translation.z()), pos_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("Z=%.2f", tf2_translation.z()), pos_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      pos_text_pos.y += line_height;
      cv::putText(cv_ptr->image, "IDs: " + ids_str, pos_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, "IDs: " + ids_str, pos_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);

      cv::Point ori_text_pos(cv_ptr->image.cols - 150, 10 + line_height);
      cv::putText(cv_ptr->image, "ORIENTATION", ori_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, "ORIENTATION", ori_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      ori_text_pos.y += line_height;
      cv::putText(cv_ptr->image, cv::format("X=%.2f", quat.getX()), ori_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("X=%.2f", quat.getX()), ori_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      ori_text_pos.y += line_height;
      cv::putText(cv_ptr->image, cv::format("Y=%.2f", quat.getY()), ori_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("Y=%.2f", quat.getY()), ori_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      ori_text_pos.y += line_height;
      cv::putText(cv_ptr->image, cv::format("Z=%.2f", quat.getZ()), ori_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("Z=%.2f", quat.getZ()), ori_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
      ori_text_pos.y += line_height;
      cv::putText(cv_ptr->image, cv::format("W=%.2f", quat.getW()), ori_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("W=%.2f", quat.getW()), ori_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);

      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = msg->header.stamp;
      transform.header.frame_id = msg->header.frame_id;
      transform.child_frame_id = "marker_frame";
      transform.transform.translation.x = tf2_translation.x();
      transform.transform.translation.y = tf2_translation.y();
      transform.transform.translation.z = tf2_translation.z();
      transform.transform.rotation.x = quat.getX();
      transform.transform.rotation.y = quat.getY();
      transform.transform.rotation.z = quat.getZ();
      transform.transform.rotation.w = quat.getW();

      tf_broadcaster_->sendTransform(transform);
    }
    else if ((now - last_pose_failed_log_).seconds() >= 1.0)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Fractal marker found, but pose estimation failed: frames=%zu frame_id=%s",
        frame_count_, msg->header.frame_id.c_str());
      last_pose_failed_log_ = now;
    }
  }
  else
  {
    last_detected_ids_str_ = "None";
    if ((now - last_no_detection_log_).seconds() >= 2.0)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "No fractal marker yet: frames=%zu image=%dx%d encoding=%s frame_id=%s "
        "(normal before the UAV reaches the pad and the gimbal points down)",
        frame_count_, cv_ptr->image.cols, cv_ptr->image.rows,
        msg->encoding.c_str(), msg->header.frame_id.c_str());
      last_no_detection_log_ = now;
    }

    cv::putText(cv_ptr->image, "NOT FOUND", cv::Point(20, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    cv::putText(cv_ptr->image, "NOT FOUND", cv::Point(20, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  }

  const auto callback_end = std::chrono::steady_clock::now();
  last_processing_latency_ms_ =
    std::chrono::duration<double, std::milli>(callback_end - callback_start).count();

  source_latency_valid_ = false;
  const rclcpp::Time input_stamp(msg->header.stamp, this->get_clock()->get_clock_type());
  if (input_stamp.nanoseconds() > 0)
  {
    const double source_latency_ms = (this->get_clock()->now() - input_stamp).seconds() * 1000.0;
    // Reject incompatible clock domains instead of displaying a misleading value.
    if (source_latency_ms >= -1.0 && source_latency_ms < 60000.0)
    {
      last_source_latency_ms_ = source_latency_ms;
      source_latency_valid_ = true;
    }
  }

  if (show_latency_overlay_)
  {
    drawLatencyOverlay(cv_ptr->image);
  }

  if ((now - last_latency_log_).seconds() >= 1.0)
  {
    if (source_latency_valid_)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Tracker latency: source_to_tracker=%.1fms processing=%.1fms threshold=%.1fms",
        last_source_latency_ms_, last_processing_latency_ms_, latency_warn_ms_);
    }
    else
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Tracker latency: source_to_tracker=N/A (camera and node clocks differ), "
        "processing=%.1fms",
        last_processing_latency_ms_);
    }
    last_latency_log_ = now;
  }

  try 
  {
    image_pub_->publish(*cv_ptr->toImageMsg());
  } 
  catch (cv_bridge::Exception& e) 
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
    return;
  }
}

void ArucoFractalTracker::drawLatencyOverlay(cv::Mat& image) const
{
  const int font_face = cv::FONT_HERSHEY_SIMPLEX;
  const double font_scale = 0.55;
  const int thickness = 1;
  const int margin = 10;
  const int line_height = 22;
  const int panel_height = 3 * line_height + 12;
  const int panel_top = std::max(0, image.rows - panel_height - margin);
  const int panel_right = std::min(image.cols - margin, 510);

  cv::rectangle(
    image, cv::Point(margin, panel_top), cv::Point(panel_right, image.rows - margin),
    cv::Scalar(0, 0, 0), cv::FILLED);

  std::string processing_text = cv::format("Detector processing: %.1f ms", last_processing_latency_ms_);
  if (last_processing_latency_ms_ > latency_warn_ms_)
  {
    processing_text += "  [WARN]";
  }
  const cv::Scalar processing_color =
    last_processing_latency_ms_ <= latency_warn_ms_
    ? cv::Scalar(80, 220, 80)
    : cv::Scalar(0, 80, 255);
  cv::putText(
    image, processing_text,
    cv::Point(margin + 8, panel_top + line_height),
    font_face, font_scale, processing_color, thickness, cv::LINE_AA);

  std::string source_text = "Camera -> tracker: N/A (clock mismatch)";
  if (source_latency_valid_)
  {
    source_text = cv::format("Camera -> tracker: %.1f ms", last_source_latency_ms_);
    if (last_source_latency_ms_ > latency_warn_ms_)
    {
      source_text += "  [WARN]";
    }
  }
  const cv::Scalar source_color = !source_latency_valid_
    ? cv::Scalar(0, 200, 255)
    : (last_source_latency_ms_ <= latency_warn_ms_
        ? cv::Scalar(80, 220, 80)
        : cv::Scalar(0, 80, 255));
  cv::putText(
    image, source_text, cv::Point(margin + 8, panel_top + 2 * line_height),
    font_face, font_scale, source_color, thickness, cv::LINE_AA);

  std::string info_text = cv::format("Tracker FPS: %.1f Hz | Detected IDs: %s", current_fps_, last_detected_ids_str_.c_str());
  cv::putText(
    image, info_text, cv::Point(margin + 8, panel_top + 3 * line_height),
    font_face, font_scale, cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
}

} // namespace fractal_tracker

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fractal_tracker::ArucoFractalTracker)
