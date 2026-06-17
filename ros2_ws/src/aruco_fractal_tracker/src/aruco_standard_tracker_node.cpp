#include "aruco_fractal_tracker/aruco_standard_tracker_node.hpp"

#include <stdexcept>
#include <string>
#include <chrono>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace standard_tracker
{

static cv::Ptr<cv::aruco::Dictionary> get_dictionary_by_name(const std::string& name) {
  if (name == "DICT_4X4_50") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  if (name == "DICT_4X4_100") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
  if (name == "DICT_4X4_250") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  if (name == "DICT_4X4_1000") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
  if (name == "DICT_5X5_50") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
  if (name == "DICT_5X5_100") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
  if (name == "DICT_5X5_250") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
  if (name == "DICT_5X5_1000") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
  if (name == "DICT_6X6_50") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
  if (name == "DICT_6X6_100") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
  if (name == "DICT_6X6_250") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  if (name == "DICT_6X6_1000") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
  if (name == "DICT_7X7_50") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
  if (name == "DICT_7X7_100") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
  if (name == "DICT_7X7_250") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
  if (name == "DICT_7X7_1000") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
  if (name == "DICT_ARUCO_ORIGINAL") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  if (name == "DICT_APRILTAG_16h5") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
  if (name == "DICT_APRILTAG_25h9") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);
  if (name == "DICT_APRILTAG_36h10") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h10);
  if (name == "DICT_APRILTAG_36h11") return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  throw std::invalid_argument("Unknown dictionary name: " + name);
}

ArucoStandardTracker::ArucoStandardTracker(const rclcpp::NodeOptions &options)
  : Node("aruco_standard_tracker", options)
{
  this->declare_parameter<std::string>("image_topic", "/gimbal_camera");
  this->declare_parameter<std::string>("camera_info_topic", "/gimbal_camera/camera_info");
  this->declare_parameter<std::string>("pose_output_topic", "/aruco_tracker/pose");
  this->declare_parameter<std::string>("annotated_image_topic", "/landing/annotated_image");
  this->declare_parameter<std::string>("dictionary", "DICT_4X4_50");
  this->declare_parameter<int>("target_tag_id", 0);
  this->declare_parameter<double>("marker_size", 0.50);
  this->declare_parameter<std::string>("camera_frame", "camera_link");

  this->declare_parameter<double>("camera_x_to_body_east_sign", 1.0);
  this->declare_parameter<double>("camera_y_to_body_north_sign", -1.0);
  this->declare_parameter<double>("camera_offset_x", 0.1517);
  this->declare_parameter<double>("camera_offset_y", 0.0);
  this->declare_parameter<bool>("show_latency_overlay", true);
  this->declare_parameter<double>("latency_warn_ms", 100.0);

  auto image_topic = this->get_parameter("image_topic").as_string();
  auto info_topic = this->get_parameter("camera_info_topic").as_string();
  auto pose_topic = this->get_parameter("pose_output_topic").as_string();
  auto annotated_topic = this->get_parameter("annotated_image_topic").as_string();
  dictionary_name_ = this->get_parameter("dictionary").as_string();
  target_tag_id_ = this->get_parameter("target_tag_id").as_int();
  marker_size_ = this->get_parameter("marker_size").as_double();
  camera_frame_ = this->get_parameter("camera_frame").as_string();

  camera_x_to_east_sign_ = this->get_parameter("camera_x_to_body_east_sign").as_double();
  camera_y_to_north_sign_ = this->get_parameter("camera_y_to_body_north_sign").as_double();
  camera_offset_x_ = this->get_parameter("camera_offset_x").as_double();
  camera_offset_y_ = this->get_parameter("camera_offset_y").as_double();
  show_latency_overlay_ = this->get_parameter("show_latency_overlay").as_bool();
  latency_warn_ms_ = this->get_parameter("latency_warn_ms").as_double();

  // Load ArUco dictionary and detector params
  dictionary_ = get_dictionary_by_name(dictionary_name_);
  detector_params_ = cv::aruco::DetectorParameters::create();
  detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  detector_params_->minMarkerPerimeterRate = 0.01;

  // Fallback nominal camera matrix if info topic not yet received (HFOV: 1.4137 rad, size: 1280x720)
  initFallbackIntrinsics();

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    info_topic, 10, std::bind(&ArucoStandardTracker::cameraInfoCallback, this, std::placeholders::_1));

  // Queue size is 1 to drop older frames when bsy
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic, 1, std::bind(&ArucoStandardTracker::imageCallback, this, std::placeholders::_1));

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(annotated_topic, 10);
  marker_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

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

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  last_no_detection_log_ = this->get_clock()->now();
  last_pose_log_ = this->get_clock()->now();
  last_pose_failed_log_ = this->get_clock()->now();
  last_latency_log_ = this->get_clock()->now();
  last_fps_time_ = this->get_clock()->now();

  RCLCPP_INFO(
    this->get_logger(),
    "ArucoStandardTracker ready: dictionary=%s target_tag_id=%d marker_size=%.3f "
    "latency_overlay=%s warn=%.1fms",
    dictionary_name_.c_str(), target_tag_id_, marker_size_,
    show_latency_overlay_ ? "on" : "off", latency_warn_ms_);
}

void ArucoStandardTracker::initFallbackIntrinsics()
{
  camera_matrix_ = (cv::Mat_<double>(3, 3) << 
    749.338, 0.0, 640.0,
    0.0, 749.338, 360.0,
    0.0, 0.0, 1.0);
  dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
}

void ArucoStandardTracker::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
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

  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i)
  {
    camera_matrix_.at<double>(i / 3, i % 3) = msg->k[i];
  }

  dist_coeffs_ = cv::Mat(static_cast<int>(msg->d.size()), 1, CV_64F);
  for (size_t i = 0; i < msg->d.size(); ++i)
  {
    dist_coeffs_.at<double>(static_cast<int>(i), 0) = msg->d[i];
  }

  RCLCPP_INFO(
    this->get_logger(),
    "CameraInfo updated: size=%ux%u fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
    msg->width, msg->height, msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
}

void ArucoStandardTracker::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
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
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } 
  catch (cv_bridge::Exception& e) 
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
    return;
  }

  cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

  // Detect markers
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners, rejected;
  cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_, rejected);

  // Check if target tag is found
  bool target_found = false;
  if (!ids.empty())
  {
    for (int id : ids)
    {
      if (id == target_tag_id_)
      {
        target_found = true;
        break;
      }
    }
  }

  // Double-pass Otsu threshold fallback if standard detection fails
  if (!target_found)
  {
    cv::Mat thresh;
    cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    std::vector<int> ids_t;
    std::vector<std::vector<cv::Point2f>> corners_t, rejected_t;
    cv::aruco::detectMarkers(thresh, dictionary_, corners_t, ids_t, detector_params_, rejected_t);
    
    if (!ids_t.empty())
    {
      for (int id : ids_t)
      {
        if (id == target_tag_id_)
        {
          corners = corners_t;
          ids = ids_t;
          rejected = rejected_t;
          target_found = true;
          break;
        }
      }
    }
  }

  std::vector<std::vector<cv::Point2f>> target_corners;
  if (target_found)
  {
    // Draw all detected markers
    cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

    // Extract target marker corners
    for (size_t i = 0; i < ids.size(); ++i)
    {
      if (ids[i] == target_tag_id_)
      {
        target_corners.push_back(corners[i]);
        break;
      }
    }
  }

  std::string ids_str = "";
  if (!ids.empty())
  {
    for (size_t i = 0; i < ids.size(); ++i)
    {
      if (i > 0) ids_str += ",";
      ids_str += std::to_string(ids[i]);
    }
  }
  last_detected_ids_str_ = ids_str.empty() ? "None" : ids_str;

  bool pose_valid = false;
  tf2::Vector3 tf2_translation(0, 0, 0);
  tf2::Quaternion quat(0, 0, 0, 1);

  if (!target_corners.empty())
  {
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(target_corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);
    
    if (!rvecs.empty() && !tvecs.empty())
    {
      pose_valid = true;
      cv::Vec3d rvec = rvecs[0];
      cv::Vec3d tvec = tvecs[0];

      // Draw axis
      cv::aruco::drawAxis(cv_ptr->image, camera_matrix_, dist_coeffs_, rvec, tvec, marker_size_ * 0.5f);

      cv::Mat rmatrix;
      cv::Rodrigues(rvec, rmatrix);
      tf2::Matrix3x3 tf2_rot(rmatrix.at<double>(0, 0), rmatrix.at<double>(0, 1), rmatrix.at<double>(0, 2),
                             rmatrix.at<double>(1, 0), rmatrix.at<double>(1, 1), rmatrix.at<double>(1, 2),
                             rmatrix.at<double>(2, 0), rmatrix.at<double>(2, 1), rmatrix.at<double>(2, 2));
        
      tf2_translation = tf2::Vector3(tvec[0], tvec[1], tvec[2]);
      tf2_rot.getRotation(quat);

      // Publish PoseStamped
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = camera_frame_;
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
          "Marker ID %d detected: frames=%zu detections=%zu tvec=[%.2f, %.2f, %.2f] ids=[%s] frame_id=%s",
          target_tag_id_, frame_count_, detection_count_,
          tf2_translation.x(), tf2_translation.y(), tf2_translation.z(),
          ids_str.c_str(),
          camera_frame_.c_str());
        last_pose_log_ = now;
      }

      // Draw standard coordinate text overlays
      int base_line = 0;
      int font_face = cv::FONT_HERSHEY_PLAIN;
      double font_scale = 1.0;
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

      // Publish TF Transform
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = msg->header.stamp;
      transform.header.frame_id = camera_frame_;
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
        "Marker ID %d found, but pose estimation failed",
        target_tag_id_);
      last_pose_failed_log_ = now;
    }
  }
  else
  {
    if ((now - last_no_detection_log_).seconds() >= 2.0)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "No marker detected yet: frames=%zu image=%dx%d",
        frame_count_, cv_ptr->image.cols, cv_ptr->image.rows);
      last_no_detection_log_ = now;
    }

    cv::putText(cv_ptr->image, "NOT FOUND", cv::Point(20, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    cv::putText(cv_ptr->image, "NOT FOUND", cv::Point(20, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  }

  // Draw the HUD ENU coordinates and Lander State overlay.
  {
    const int font_face = cv::FONT_HERSHEY_SIMPLEX;
    const double font_scale = 0.55;
    const int thickness = 1;
    const int line_h = 22;
    const int margin = 10;
    const int panel_w = 400;
    const int panel_h = 6 * line_h + 12;
    const int panel_top = std::max(0, cv_ptr->image.rows - panel_h - margin);
    const int panel_left = std::max(0, cv_ptr->image.cols - panel_w - margin);

    cv::rectangle(
      cv_ptr->image, cv::Point(panel_left, panel_top), cv::Point(cv_ptr->image.cols - margin, cv_ptr->image.rows - margin),
      cv::Scalar(0, 0, 0), cv::FILLED);

    cv::rectangle(
      cv_ptr->image, cv::Point(panel_left, panel_top), cv::Point(cv_ptr->image.cols - margin, cv_ptr->image.rows - margin),
      cv::Scalar(150, 150, 150), 1);

    cv::Point pos(panel_left + 10, panel_top + line_h);

    auto draw_text = [&](const std::string& text, const cv::Scalar& color = cv::Scalar(255, 255, 255)) {
      cv::putText(cv_ptr->image, text, pos, font_face, font_scale, color, thickness, cv::LINE_AA);
      pos.y += line_h;
    };

    draw_text("FLIGHT STATE: " + last_lander_state_, cv::Scalar(80, 220, 240));

    if (last_uav_pose_)
    {
      double uav_x = last_uav_pose_->pose.position.x;
      double uav_y = last_uav_pose_->pose.position.y;
      double uav_z = last_uav_pose_->pose.position.z;

      tf2::Quaternion q(
        last_uav_pose_->pose.orientation.x,
        last_uav_pose_->pose.orientation.y,
        last_uav_pose_->pose.orientation.z,
        last_uav_pose_->pose.orientation.w
      );
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      draw_text(cv::format("UAV ENU: E=%.2f, N=%.2f, U=%.2f", uav_x, uav_y, uav_z));
      draw_text(cv::format("UAV YAW: %.1f deg", yaw * 180.0 / 3.141592653589793));

      if (pose_valid)
      {
        double tx = tf2_translation.x();
        double ty = tf2_translation.y();

        double east_body = camera_x_to_east_sign_ * tx;
        double north_body = camera_y_to_north_sign_ * ty;

        double x_body = north_body + camera_offset_x_;
        double y_body = -east_body + camera_offset_y_;

        double c = cos(yaw);
        double s = sin(yaw);

        double rel_east = x_body * c - y_body * s;
        double rel_north = x_body * s + y_body * c;

        double abs_east = uav_x + rel_east;
        double abs_north = uav_y + rel_north;

        draw_text(cv::format("REL ENU: E=%.2f, N=%.2f", rel_east, rel_north), cv::Scalar(100, 255, 100));
        draw_text(cv::format("TGT ENU: E=%.2f, N=%.2f", abs_east, abs_north), cv::Scalar(100, 100, 255));
        draw_text(cv::format("CAM TVEC: [%.2f, %.2f, %.2f]", tx, ty, tf2_translation.z()));
      }
      else
      {
        draw_text("REL ENU: NO MARKER DETECTED", cv::Scalar(0, 0, 255));
        draw_text("TGT ENU: NO MARKER DETECTED", cv::Scalar(0, 0, 255));
        draw_text("CAM TVEC: N/A");
      }
    }
    else
    {
      draw_text("UAV ENU: WAITING FOR MAVROS...", cv::Scalar(0, 150, 255));
      draw_text("UAV YAW: WAITING FOR MAVROS...", cv::Scalar(0, 150, 255));
      draw_text("REL ENU: WAITING FOR MAVROS...");
      draw_text("TGT ENU: WAITING FOR MAVROS...");
      draw_text("CAM TVEC: N/A");
    }
  }

  const auto callback_end = std::chrono::steady_clock::now();
  last_processing_latency_ms_ =
    std::chrono::duration<double, std::milli>(callback_end - callback_start).count();

  source_latency_valid_ = false;
  const rclcpp::Time input_stamp(msg->header.stamp, this->get_clock()->get_clock_type());
  if (input_stamp.nanoseconds() > 0)
  {
    const double source_latency_ms = (this->get_clock()->now() - input_stamp).seconds() * 1000.0;
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

  try 
  {
    image_pub_->publish(*cv_ptr->toImageMsg());
  } 
  catch (cv_bridge::Exception& e) 
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
  }
}

void ArucoStandardTracker::drawLatencyOverlay(cv::Mat& image) const
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

} // namespace standard_tracker

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(standard_tracker::ArucoStandardTracker)
