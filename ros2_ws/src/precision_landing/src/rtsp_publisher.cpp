#include "precision_landing/rtsp_publisher.hpp"
#include <cstdlib>
#include <chrono>

namespace precision_landing
{

RtspPublisher::RtspPublisher(const rclcpp::NodeOptions & options)
: Node("siyi_rtsp_publisher", options)
{
  // Set RTSP TCP transport BEFORE opening capture
  setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp", 1);

  // Declare Parameters
  this->declare_parameter<std::string>("rtsp_url", "rtsp://192.168.168.14:8554/main.264");
  this->declare_parameter<std::string>("frame_id", "siyi_camera_optical_frame");
  this->declare_parameter<bool>("flip_180", true);
  this->declare_parameter<double>("target_fps", 30.0);
  this->declare_parameter<int>("image_width", 1280);
  this->declare_parameter<int>("image_height", 720);
  this->declare_parameter<double>("camera_fx", 749.338);
  this->declare_parameter<double>("camera_fy", 749.338);
  this->declare_parameter<double>("camera_cx", 640.0);
  this->declare_parameter<double>("camera_cy", 360.0);
  this->declare_parameter<std::vector<double>>("camera_d", {0.0, 0.0, 0.0, 0.0, 0.0});

  // Get Parameters
  rtsp_url_ = this->get_parameter("rtsp_url").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  flip_180_ = this->get_parameter("flip_180").as_bool();
  target_fps_ = this->get_parameter("target_fps").as_double();
  image_width_ = this->get_parameter("image_width").as_int();
  image_height_ = this->get_parameter("image_height").as_int();
  camera_fx_ = this->get_parameter("camera_fx").as_double();
  camera_fy_ = this->get_parameter("camera_fy").as_double();
  camera_cx_ = this->get_parameter("camera_cx").as_double();
  camera_cy_ = this->get_parameter("camera_cy").as_double();
  camera_d_ = this->get_parameter("camera_d").as_double_array();

  // Create Publishers
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/siyi/image_raw", 10);
  info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/siyi/camera_info", 10);

  // Build static CameraInfo msg
  camera_info_msg_ = build_camera_info();

  // Open RTSP
  open_capture();

  // Create Timer
  double timer_period_sec = 1.0 / target_fps_;
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period_sec),
    std::bind(&RtspPublisher::timer_callback, this)
  );

  RCLCPP_INFO(
    this->get_logger(),
    "SIYI RTSP Publisher C++ started: url=%s, flip_180=%s, target_fps=%.1f, resolution=%dx%d",
    rtsp_url_.c_str(), flip_180_ ? "true" : "false", target_fps_, image_width_, image_height_
  );
}

RtspPublisher::~RtspPublisher()
{
  if (cap_.isOpened()) {
    cap_.release();
    RCLCPP_INFO(this->get_logger(), "RTSP capture released");
  }
}

bool RtspPublisher::open_capture()
{
  if (cap_.isOpened()) {
    cap_.release();
  }

  RCLCPP_INFO(this->get_logger(), "Opening RTSP stream: %s", rtsp_url_.c_str());
  cap_.open(rtsp_url_, cv::CAP_FFMPEG);

  if (!cap_.isOpened()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to open RTSP stream: %s. Will retry on next timer tick.",
      rtsp_url_.c_str()
    );
    return false;
  }

  cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
  cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

  fail_count_ = 0;
  RCLCPP_INFO(this->get_logger(), "RTSP stream opened successfully");
  return true;
}

sensor_msgs::msg::CameraInfo RtspPublisher::build_camera_info()
{
  sensor_msgs::msg::CameraInfo msg;
  msg.header.frame_id = frame_id_;
  msg.width = image_width_;
  msg.height = image_height_;
  msg.distortion_model = "plumb_bob";
  msg.d = camera_d_;

  msg.k = {
    camera_fx_, 0.0,        camera_cx_,
    0.0,        camera_fy_, camera_cy_,
    0.0,        0.0,        1.0
  };

  msg.r = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
  };

  msg.p = {
    camera_fx_, 0.0,        camera_cx_, 0.0,
    0.0,        camera_fy_, camera_cy_, 0.0,
    0.0,        0.0,        1.0,        0.0
  };

  return msg;
}

void RtspPublisher::timer_callback()
{
  if (!cap_.isOpened()) {
    fail_count_++;
    if (fail_count_ % 30 == 1) {
      RCLCPP_WARN(
        this->get_logger(),
        "RTSP not open, attempting reconnect... (fail_count=%d)",
        fail_count_
      );
    }
    open_capture();
    return;
  }

  cv::Mat frame;
  bool ret = cap_.read(frame);

  if (!ret || frame.empty()) {
    fail_count_++;
    if (fail_count_ >= max_consecutive_fails_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Lost RTSP stream after %d failures, reconnecting...",
        fail_count_
      );
      open_capture();
    }
    return;
  }

  fail_count_ = 0;
  frame_count_++;

  if (flip_180_) {
    cv::flip(frame, frame, -1);
  }

  auto stamp = this->get_clock()->now();

  // Publish Image
  try {
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id_;
    auto img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    image_pub_->publish(*img_msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    return;
  }

  // Publish CameraInfo
  camera_info_msg_.header.stamp = stamp;
  info_pub_->publish(camera_info_msg_);

  // Log stats periodically (every 5 seconds)
  if (frame_count_ % (static_cast<int>(target_fps_) * 5) == 0) {
    RCLCPP_INFO(
      this->get_logger(),
      "Published %d frames (%dx%d) flip_180=%s",
      frame_count_, frame.cols, frame.rows, flip_180_ ? "true" : "false"
    );
  }
}

}  // namespace precision_landing

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(precision_landing::RtspPublisher)
