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

#include "precision_landing/aruco_fractal_tracker_node.hpp"

#include <stdexcept>
#include <string>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <std_msgs/msg/header.hpp>

namespace fractal_tracker
{
ArucoFractalTracker::ArucoFractalTracker(const rclcpp::NodeOptions &options)
  : Node("aruco_fractal_tracker", options)
{
  this->declare_parameter<std::string>("marker_configuration", "");
  this->declare_parameter<double>("marker_size", 0.5);
  this->declare_parameter<double>("min_tracking_z", 0.15);
  this->declare_parameter<double>("max_tracking_z", 12.0);
  this->declare_parameter<double>("max_pose_jump_m", 2.0);
  this->declare_parameter<int>("acquire_good_frames", 5);
  this->declare_parameter<int>("lost_bad_frames", 3);
  this->declare_parameter<bool>("show_latency_overlay", true);
  this->declare_parameter<double>("latency_warn_ms", 100.0);
  this->declare_parameter<double>("camera_x_to_body_east_sign", -1.0);
  this->declare_parameter<double>("camera_y_to_body_north_sign", 1.0);
  this->declare_parameter<double>("camera_offset_x", 0.1517);
  this->declare_parameter<double>("camera_offset_y", 0.0);
  this->declare_parameter<std::string>("box_telemetry_topic", "/b1/telemetry");

  auto marker_configuration = this->get_parameter("marker_configuration").get_value<std::string>();
  marker_size_ = this->get_parameter("marker_size").get_value<double>();
  min_tracking_z_ = this->get_parameter("min_tracking_z").as_double();
  max_tracking_z_ = this->get_parameter("max_tracking_z").as_double();
  max_pose_jump_m_ = this->get_parameter("max_pose_jump_m").as_double();
  acquire_good_frames_ = this->get_parameter("acquire_good_frames").as_int();
  lost_bad_frames_ = this->get_parameter("lost_bad_frames").as_int();
  show_latency_overlay_ = this->get_parameter("show_latency_overlay").as_bool();
  latency_warn_ms_ = this->get_parameter("latency_warn_ms").as_double();
  camera_x_to_east_sign_ = this->get_parameter("camera_x_to_body_east_sign").as_double();
  camera_y_to_north_sign_ = this->get_parameter("camera_y_to_body_north_sign").as_double();
  camera_offset_x_ = this->get_parameter("camera_offset_x").as_double();
  camera_offset_y_ = this->get_parameter("camera_offset_y").as_double();
  const auto box_telemetry_topic = this->get_parameter("box_telemetry_topic").as_string();

  detector_.setConfiguration(marker_configuration);

  // Set default/fallback camera parameters (width: 1280, height: 720, HFOV: 1.4137 rad / 81 degrees)
  // fx = fy = (1280 / 2) / tan(1.4137 / 2) = 640 / tan(40.5 deg) = 749.338
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    749.338, 0.0, 640.0,
    0.0, 749.338, 360.0,
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
  target_pub_ = this->create_publisher<dib_msgs::msg::LandingTarget6D>("target_output_topic", 10);

  rclcpp::QoS pose_qos(1);
  pose_qos.best_effort();
  pose_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  uav_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", pose_qos,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      last_uav_pose_ = msg;
    });

  lander_state_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/lander/state", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      last_lander_state_ = msg->data;
    });

  box_telemetry_sub_ = this->create_subscription<dib_msgs::msg::BoxTelemetry>(
    box_telemetry_topic, 10,
    [this](const dib_msgs::msg::BoxTelemetry::SharedPtr msg) {
      last_box_yaw_ = static_cast<double>(msg->box_info.yaw);
      last_box_yaw_valid_ = std::isfinite(last_box_yaw_);
    });

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  last_no_detection_log_ = this->get_clock()->now();
  last_pose_log_ = this->get_clock()->now();
  last_pose_failed_log_ = this->get_clock()->now();
  last_latency_log_ = this->get_clock()->now();
  last_fps_time_ = this->get_clock()->now();

  getSystemCPUStats(last_sys_idle_, last_sys_total_);
  getProcessCPUStats(last_proc_ticks_);

  RCLCPP_INFO(
    this->get_logger(),
    "ArucoFractalTracker ready: marker_configuration=%s marker_size=%.3f "
    "quality_z=[%.2f, %.2f] max_jump=%.2fm acquire=%d lost=%d "
    "latency_overlay=%s warn=%.1fms",
    marker_configuration.c_str(), marker_size_,
    min_tracking_z_, max_tracking_z_, max_pose_jump_m_,
    acquire_good_frames_, lost_bad_frames_,
    show_latency_overlay_ ? "on" : "off", latency_warn_ms_);
  RCLCPP_INFO(
    this->get_logger(),
    "Topics before remap: image_input_topic -> image_output_topic, poses_output_topic; "
    "target_output_topic publishes LandingTarget6D; CameraInfo is optional because fallback intrinsics are loaded");
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

    long sys_idle = 0, sys_total = 0;
    if (getSystemCPUStats(sys_idle, sys_total))
    {
      long total_diff = sys_total - last_sys_total_;
      long idle_diff = sys_idle - last_sys_idle_;
      if (total_diff > 0)
      {
        system_cpu_usage_ = 100.0 * (total_diff - idle_diff) / total_diff;
      }
      last_sys_total_ = sys_total;
      last_sys_idle_ = sys_idle;
    }

    long proc_ticks = 0;
    if (getProcessCPUStats(proc_ticks))
    {
      long ticks_diff = proc_ticks - last_proc_ticks_;
      long clk_tck = sysconf(_SC_CLK_TCK);
      int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
      if (clk_tck > 0 && num_cores > 0)
      {
        double usage_single_core = 100.0 * (double(ticks_diff) / clk_tck) / elapsed;
        process_cpu_usage_ = usage_single_core / num_cores;
      }
      last_proc_ticks_ = proc_ticks;
    }
  }

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
    return;
  }

  cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

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
      const double marker_distance_m = tf2_translation.length();
      last_marker_distance_ = marker_distance_m;
      last_marker_distance_valid_ = true;

      std::string reject_reason;
      const bool pose_accepted = acceptPose(tf2_translation, reject_reason);

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
      publishTarget(msg->header, tf2_translation, tf2_rot, markers.empty() ? 0 : markers.front().id);

      if ((now - last_pose_log_).seconds() >= 1.0)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Fractal marker detected: frames=%zu detections=%zu state=%u accepted=%s tvec=[%.2f, %.2f, %.2f] ids=[%s] frame_id=%s%s%s",
          frame_count_, detection_count_,
          tracking_state_, pose_accepted ? "yes" : "no",
          tf2_translation.x(), tf2_translation.y(), tf2_translation.z(),
          ids_str.c_str(),
          msg->header.frame_id.c_str(),
          reject_reason.empty() ? "" : " reject=",
          reject_reason.c_str());
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
      cv::putText(cv_ptr->image, cv::format("DIST=%.2fm", marker_distance_m), pos_text_pos, font_face, font_scale, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      cv::putText(cv_ptr->image, cv::format("DIST=%.2fm", marker_distance_m), pos_text_pos, font_face, font_scale, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
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

      // Removed old HUD overlay block to be drawn at the end of callback

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
    else
    {
      last_marker_distance_valid_ = false;
      publishTrackerStateOnly(msg->header);
      if ((now - last_pose_failed_log_).seconds() >= 1.0)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Fractal marker found, but pose estimation failed: frames=%zu frame_id=%s",
          frame_count_, msg->header.frame_id.c_str());
        last_pose_failed_log_ = now;
      }
    }
  }
  else
  {
    last_marker_distance_valid_ = false;
    last_detected_ids_str_ = "None";
    publishTrackerStateOnly(msg->header);
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

  // Draw the HUD ENU coordinates and Lander State overlay.
  // This is drawn at the bottom-right (X = image.cols - 450) to avoid overlapping
  // with the latency overlay at the bottom-left.
  {
    const int font_face = cv::FONT_HERSHEY_SIMPLEX;
    const double font_scale = 0.55;
    const int thickness = 1;
    const int line_h = 22;
    const int margin = 10;
    const int panel_w = 400;
    const int panel_h = 8 * line_h + 12;
    const int panel_top = std::max(0, cv_ptr->image.rows - panel_h - margin);
    const int panel_left = std::max(0, cv_ptr->image.cols - panel_w - margin);

    // Draw semi-transparent background panel
    drawTransparentRect(
      cv_ptr->image, cv::Rect(panel_left, panel_top, cv_ptr->image.cols - margin - panel_left, cv_ptr->image.rows - margin - panel_top),
      cv::Scalar(10, 10, 15), 0.65);

    // Draw cyan border around the panel
    cv::rectangle(
      cv_ptr->image, cv::Point(panel_left, panel_top), cv::Point(cv_ptr->image.cols - margin, cv_ptr->image.rows - margin),
      cv::Scalar(0, 220, 220), 1);

    cv::Point pos(panel_left + 10, panel_top + line_h);

    auto draw_text = [&](const std::string& text, const cv::Scalar& color = cv::Scalar(255, 255, 255)) {
      cv::putText(cv_ptr->image, text, pos, font_face, font_scale, color, thickness, cv::LINE_AA);
      pos.y += line_h;
    };

    draw_text("FLIGHT STATE: " + last_lander_state_, cv::Scalar(80, 220, 240)); // Cyan color

    bool marker_pose_valid = false;
    double marker_tx = 0.0;
    double marker_ty = 0.0;
    double marker_tz = 0.0;
    double marker_distance_m = 0.0;

    if (detector_.poseEstimation())
    {
      cv::Mat tvec = detector_.getTvec();
      marker_tx = tvec.at<double>(0, 0);
      marker_ty = tvec.at<double>(1, 0);
      marker_tz = tvec.at<double>(2, 0);
      marker_distance_m = std::sqrt(
        marker_tx * marker_tx + marker_ty * marker_ty + marker_tz * marker_tz);
      marker_pose_valid = true;
    }

    if (last_uav_pose_)
    {
      double uav_x = last_uav_pose_->pose.position.x; // East
      double uav_y = last_uav_pose_->pose.position.y; // North
      double uav_z = last_uav_pose_->pose.position.z; // Up

      tf2::Quaternion q(
        last_uav_pose_->pose.orientation.x,
        last_uav_pose_->pose.orientation.y,
        last_uav_pose_->pose.orientation.z,
        last_uav_pose_->pose.orientation.w
      );
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw); // yaw is ENU

      draw_text(cv::format("UAV ENU: E=%.2f, N=%.2f, U=%.2f", uav_x, uav_y, uav_z));
      draw_text(cv::format("UAV YAW: %.1f deg", yaw * 180.0 / 3.141592653589793));
      if (last_box_yaw_valid_)
      {
        const double pi = 3.14159265358979323846;
        const double delta_yaw = std::atan2(std::sin(last_box_yaw_ - yaw), std::cos(last_box_yaw_ - yaw));
        draw_text(
          cv::format(
            "BOX YAW: %.1f deg | dYAW: %.1f deg",
            last_box_yaw_ * 180.0 / pi,
            delta_yaw * 180.0 / pi),
          std::abs(delta_yaw) <= 5.0 * pi / 180.0 ? cv::Scalar(80, 255, 80) : cv::Scalar(0, 200, 255));
      }
      else
      {
        draw_text("BOX YAW: N/A | dYAW: N/A", cv::Scalar(0, 150, 255));
      }

      if (marker_pose_valid)
      {
        double east_body = camera_x_to_east_sign_ * marker_tx;
        double north_body = camera_y_to_north_sign_ * marker_ty;

        double x_body = north_body + camera_offset_x_;
        double y_body = -east_body + camera_offset_y_;

        double c = cos(yaw);
        double s = sin(yaw);

        double rel_east = x_body * c - y_body * s;
        double rel_north = x_body * s + y_body * c;

        double abs_east = uav_x + rel_east;
        double abs_north = uav_y + rel_north;

        draw_text(cv::format("REL ENU: E=%.2f, N=%.2f", rel_east, rel_north), cv::Scalar(100, 255, 100)); // Light green
        draw_text(cv::format("TGT ENU: E=%.2f, N=%.2f", abs_east, abs_north), cv::Scalar(100, 100, 255)); // Light red/blue
        draw_text(cv::format("MARKER DIST: %.2fm", marker_distance_m), cv::Scalar(100, 255, 255));
        draw_text(cv::format("CAM TVEC: [%.2f, %.2f, %.2f]", marker_tx, marker_ty, marker_tz));
      }
      else
      {
        draw_text("REL ENU: NO MARKER DETECTED", cv::Scalar(0, 0, 255)); // Red
        draw_text("TGT ENU: NO MARKER DETECTED", cv::Scalar(0, 0, 255));
        draw_text("MARKER DIST: N/A");
        draw_text("CAM TVEC: N/A");
      }
    }
    else
    {
      draw_text("UAV ENU: WAITING FOR MAVROS...", cv::Scalar(0, 150, 255)); // Orange/Yellow
      draw_text("UAV YAW: WAITING FOR MAVROS...", cv::Scalar(0, 150, 255));
      draw_text("BOX YAW: N/A | dYAW: N/A", cv::Scalar(0, 150, 255));
      draw_text("REL ENU: WAITING FOR MAVROS...");
      if (marker_pose_valid)
      {
        draw_text(cv::format("MARKER DIST: %.2fm", marker_distance_m), cv::Scalar(100, 255, 255));
        draw_text(cv::format("CAM TVEC: [%.2f, %.2f, %.2f]", marker_tx, marker_ty, marker_tz));
      }
      else
      {
        draw_text("MARKER DIST: N/A");
        draw_text("CAM TVEC: N/A");
      }
    }
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

bool ArucoFractalTracker::acceptPose(
  const tf2::Vector3& tvec,
  std::string& reject_reason)
{
  const bool finite =
    std::isfinite(tvec.x()) && std::isfinite(tvec.y()) && std::isfinite(tvec.z());
  if (!finite)
  {
    reject_reason = "non_finite";
  }
  else if (tvec.z() < min_tracking_z_ || tvec.z() > max_tracking_z_)
  {
    reject_reason = "z_out_of_range";
  }
  else if (have_last_tvec_ && (tvec - last_tvec_).length() > max_pose_jump_m_)
  {
    reject_reason = "pose_jump";
  }

  const bool accepted = reject_reason.empty();
  if (accepted)
  {
    ++good_frame_count_;
    bad_frame_count_ = 0;
    last_tvec_ = tvec;
    have_last_tvec_ = true;
    tracking_state_ =
      good_frame_count_ >= acquire_good_frames_
        ? dib_msgs::msg::LandingTarget6D::TRACKING
        : dib_msgs::msg::LandingTarget6D::SEARCHING;
  }
  else
  {
    good_frame_count_ = 0;
    ++bad_frame_count_;
    if (bad_frame_count_ >= lost_bad_frames_)
    {
      tracking_state_ = dib_msgs::msg::LandingTarget6D::LOST;
      have_last_tvec_ = false;
    }
    else
    {
      tracking_state_ = dib_msgs::msg::LandingTarget6D::SEARCHING;
    }
  }
  return accepted;
}

void ArucoFractalTracker::publishTarget(
  const std_msgs::msg::Header& header,
  const tf2::Vector3& tvec,
  const tf2::Matrix3x3& rotation,
  int32_t tag_id)
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  rotation.getRPY(roll, pitch, yaw);

  // Tính toán góc Yaw của Tag trong hệ tọa độ drone body (FLU)
  // Tránh suy biến Euler (Gimbal Lock) bằng cách tính trực tiếp từ cột ma trận quay
  double stable_yaw = std::atan2(-rotation[0][0], -rotation[1][0]);

  dib_msgs::msg::LandingTarget6D target;
  target.header = header;
  target.x = tvec.x();
  target.y = tvec.y();
  target.z = tvec.z();
  target.roll = roll;
  target.pitch = pitch;
  target.yaw = stable_yaw;
  target.state = tracking_state_;
  target.tag_id = tag_id;
  target_pub_->publish(target);
}

void ArucoFractalTracker::publishTrackerStateOnly(const std_msgs::msg::Header& header)
{
  good_frame_count_ = 0;
  ++bad_frame_count_;
  if (bad_frame_count_ >= lost_bad_frames_)
  {
    tracking_state_ = dib_msgs::msg::LandingTarget6D::LOST;
    have_last_tvec_ = false;
  }
  else if (tracking_state_ == dib_msgs::msg::LandingTarget6D::TRACKING)
  {
    tracking_state_ = dib_msgs::msg::LandingTarget6D::SEARCHING;
  }

  dib_msgs::msg::LandingTarget6D target;
  target.header = header;
  target.state = tracking_state_;
  target.tag_id = -1;
  target_pub_->publish(target);
}

bool ArucoFractalTracker::getSystemCPUStats(long &idle, long &total) const
{
  std::ifstream file("/proc/stat");
  if (!file.is_open())
    return false;
  std::string line;
  if (std::getline(file, line))
  {
    std::istringstream ss(line);
    std::string cpu;
    ss >> cpu;
    if (cpu == "cpu")
    {
      long user, nice, system, idle_time, iowait, irq, softirq, steal, guest, guest_nice;
      if (ss >> user >> nice >> system >> idle_time >> iowait >> irq >> softirq >> steal >> guest >> guest_nice)
      {
        idle = idle_time + iowait;
        total = user + nice + system + idle + irq + softirq + steal;
        return true;
      }
    }
  }
  return false;
}

bool ArucoFractalTracker::getProcessCPUStats(long &proc_ticks) const
{
  std::ifstream file("/proc/self/stat");
  if (!file.is_open())
    return false;
  std::string line;
  if (std::getline(file, line))
  {
    size_t last_paren = line.rfind(')');
    if (last_paren == std::string::npos)
      return false;
    std::string rest = line.substr(last_paren + 1);
    std::istringstream ss(rest);
    std::vector<std::string> tokens;
    std::string token;
    while (ss >> token)
    {
      tokens.push_back(token);
    }
    if (tokens.size() > 12)
    {
      try
      {
        long utime = std::stol(tokens[11]);
        long stime = std::stol(tokens[12]);
        proc_ticks = utime + stime;
        return true;
      }
      catch (...)
      {
        return false;
      }
    }
  }
  return false;
}

void ArucoFractalTracker::drawTransparentRect(cv::Mat& image, const cv::Rect& rect, const cv::Scalar& color, double alpha) const
{
  if (rect.x < 0 || rect.y < 0 || rect.x + rect.width > image.cols || rect.y + rect.height > image.rows)
    return;
  cv::Mat roi = image(rect);
  cv::Mat color_rect(roi.size(), roi.type(), color);
  cv::addWeighted(color_rect, alpha, roi, 1.0 - alpha, 0.0, roi);
}

void ArucoFractalTracker::drawLatencyOverlay(cv::Mat& image)
{
  const int font_face = cv::FONT_HERSHEY_SIMPLEX;
  const double font_scale = 0.55;
  const int thickness = 1;
  const int margin = 10;
  const int line_height = 22;
  const int panel_height = 4 * line_height + 12;
  const int panel_top = std::max(0, image.rows - panel_height - margin);
  const int panel_right = std::min(image.cols - margin, 520);

  // Draw semi-transparent background panel for premium diagnostics HUD
  drawTransparentRect(image, cv::Rect(margin, panel_top, panel_right - margin, image.rows - margin - panel_top), cv::Scalar(10, 10, 15), 0.65);
  // Add a nice thin cyan border
  cv::rectangle(
    image, cv::Point(margin, panel_top), cv::Point(panel_right, image.rows - margin),
    cv::Scalar(0, 220, 220), 1);

  std::string processing_text = cv::format("Detector processing: %.1f ms", last_processing_latency_ms_);
  if (last_processing_latency_ms_ > latency_warn_ms_)
  {
    processing_text += "  [WARN]";
  }
  const cv::Scalar processing_color =
    last_processing_latency_ms_ <= latency_warn_ms_
    ? cv::Scalar(80, 255, 80)
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
        ? cv::Scalar(80, 255, 80)
        : cv::Scalar(0, 80, 255));
  cv::putText(
    image, source_text, cv::Point(margin + 8, panel_top + 2 * line_height),
    font_face, font_scale, source_color, thickness, cv::LINE_AA);

  std::string info_text = cv::format("FPS: %.1f Hz | CPU: %.1f%% (Sys) | %.1f%% (Node)",
    current_fps_, system_cpu_usage_, process_cpu_usage_);
  cv::putText(
    image, info_text, cv::Point(margin + 8, panel_top + 3 * line_height),
    font_face, font_scale, cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);

  std::string dist_ids_text;
  cv::Scalar dist_ids_color;
  if (last_marker_distance_valid_)
  {
    dist_ids_text = cv::format("Marker Dist: %.2f m | IDs: %s", last_marker_distance_, last_detected_ids_str_.c_str());
    dist_ids_color = cv::Scalar(100, 255, 255);
  }
  else
  {
    dist_ids_text = "Marker Dist: N/A | IDs: None";
    dist_ids_color = cv::Scalar(150, 150, 150);
  }
  cv::putText(
    image, dist_ids_text, cv::Point(margin + 8, panel_top + 4 * line_height),
    font_face, font_scale, dist_ids_color, thickness, cv::LINE_AA);
}

} // namespace fractal_tracker

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fractal_tracker::ArucoFractalTracker)
