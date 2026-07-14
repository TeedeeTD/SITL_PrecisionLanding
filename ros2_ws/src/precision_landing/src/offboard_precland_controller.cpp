#include "precision_landing/offboard_precland_controller.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace precision_landing
{

OffboardPreclandController::OffboardPreclandController(const rclcpp::NodeOptions & options)
: Node("offboard_precland_controller", options)
{
  // --- Declare Parameters without default values (must be provided via YAML) ---
  this->declare_parameter<double>("camera_x_to_body_east_sign");
  this->declare_parameter<double>("camera_y_to_body_north_sign");
  this->declare_parameter<std::string>("camera_yaw_frame");
  this->declare_parameter<double>("camera_offset_x");
  this->declare_parameter<double>("camera_offset_y");
  this->declare_parameter<double>("camera_offset_z");
  this->declare_parameter<double>("marker_size");
  this->declare_parameter<std::string>("target_topic");
  this->declare_parameter<std::string>("target_pose_topic");
  this->declare_parameter<bool>("align_yaw_to_tag");
  this->declare_parameter<double>("tag_yaw_sign");
  this->declare_parameter<double>("tag_yaw_offset");
  this->declare_parameter<int>("land_mode");
  this->declare_parameter<double>("abort_alt");
  this->declare_parameter<double>("final_alt");
  this->declare_parameter<double>("yaw_slew_rate");
  this->declare_parameter<int>("yaw_lock_samples");
  this->declare_parameter<double>("yaw_lock_alt");
  this->declare_parameter<double>("yaw_lock_alt_2");

  // Tunable Controller Constants
  this->declare_parameter<int>("ctrl_hz");
  this->declare_parameter<double>("target_timeout");
  this->declare_parameter<int>("tracking_confirm");
  this->declare_parameter<int>("align_confirm");
  this->declare_parameter<double>("align_timeout");
  this->declare_parameter<double>("search_timeout");
  this->declare_parameter<int>("max_search");
  this->declare_parameter<double>("descent_rate");
  this->declare_parameter<double>("fappr_alt");
  this->declare_parameter<double>("hacc_rad");
  this->declare_parameter<double>("max_align_step");
  this->declare_parameter<double>("max_descent_step");
  this->declare_parameter<double>("servo_gain_high");
  this->declare_parameter<double>("servo_gain_low");

  this->declare_parameter<double>("mpc_land_alt1");
  this->declare_parameter<double>("mpc_land_alt2");
  this->declare_parameter<double>("mpc_land_alt_crawl");
  this->declare_parameter<double>("mpc_z_vel_max_dn");
  this->declare_parameter<double>("mpc_land_speed");
  this->declare_parameter<double>("mpc_land_crwl");
  this->declare_parameter<double>("high_alt");
  this->declare_parameter<double>("low_alt");
  this->declare_parameter<double>("alpha_high");
  this->declare_parameter<double>("alpha_low");
  this->declare_parameter<int>("filter_window");

  this->declare_parameter<double>("align_r_min");
  this->declare_parameter<double>("align_r_max");
  this->declare_parameter<double>("align_r_gain");
  this->declare_parameter<double>("align_r_bias");
  this->declare_parameter<double>("desc_r_min");
  this->declare_parameter<double>("desc_r_max");
  this->declare_parameter<double>("desc_r_gain");
  this->declare_parameter<double>("desc_r_bias");
  this->declare_parameter<double>("reject_r_min");
  this->declare_parameter<double>("reject_r_max");
  this->declare_parameter<double>("reject_r_gain");

  this->declare_parameter<double>("target_loss_grace");
  this->declare_parameter<double>("descent_loss_hold");
  this->declare_parameter<double>("low_alt_max_err");
  this->declare_parameter<double>("search_alt");
  this->declare_parameter<double>("search_alt_max");
  this->declare_parameter<double>("final_approach_timeout");
  this->declare_parameter<double>("final_descent_rate");
  this->declare_parameter<double>("final_align_step");
  this->declare_parameter<double>("sp_vel_max");
  this->declare_parameter<double>("sp_accel_max");
  this->declare_parameter<double>("yaw_lock_timeout");
  this->declare_parameter<int>("yaw_lock_min_samples");
  this->declare_parameter<double>("camera_mount_roll");
  this->declare_parameter<double>("camera_mount_pitch");
  this->declare_parameter<double>("camera_mount_yaw");

  // --- Get Parameters ---
  camera_x_to_body_east_sign_ = this->get_parameter("camera_x_to_body_east_sign").as_double();
  camera_y_to_body_north_sign_ = this->get_parameter("camera_y_to_body_north_sign").as_double();
  camera_yaw_frame_ = this->get_parameter("camera_yaw_frame").as_string();
  camera_offset_x_ = this->get_parameter("camera_offset_x").as_double();
  camera_offset_y_ = this->get_parameter("camera_offset_y").as_double();
  camera_offset_z_ = this->get_parameter("camera_offset_z").as_double();
  marker_size_ = this->get_parameter("marker_size").as_double();
  target_topic_ = this->get_parameter("target_topic").as_string();
  target_pose_topic_ = this->get_parameter("target_pose_topic").as_string();
  align_yaw_to_tag_ = this->get_parameter("align_yaw_to_tag").as_bool();
  tag_yaw_sign_ = this->get_parameter("tag_yaw_sign").as_double();
  tag_yaw_offset_ = this->get_parameter("tag_yaw_offset").as_double();
  land_mode_ = this->get_parameter("land_mode").as_int();
  abort_alt_param_ = this->get_parameter("abort_alt").as_double();
  final_alt_param_ = this->get_parameter("final_alt").as_double();
  yaw_slew_rate_ = this->get_parameter("yaw_slew_rate").as_double();
  yaw_lock_samples_ = this->get_parameter("yaw_lock_samples").as_int();
  yaw_lock_alt_ = this->get_parameter("yaw_lock_alt").as_double();
  yaw_lock_alt_2_ = this->get_parameter("yaw_lock_alt_2").as_double();

  ctrl_hz_ = this->get_parameter("ctrl_hz").as_int();
  target_timeout_ = this->get_parameter("target_timeout").as_double();
  tracking_confirm_ = this->get_parameter("tracking_confirm").as_int();
  align_confirm_ = this->get_parameter("align_confirm").as_int();
  align_timeout_ = this->get_parameter("align_timeout").as_double();
  search_timeout_ = this->get_parameter("search_timeout").as_double();
  max_search_ = this->get_parameter("max_search").as_int();
  descent_rate_ = this->get_parameter("descent_rate").as_double();
  fappr_alt_ = this->get_parameter("fappr_alt").as_double();
  hacc_rad_ = this->get_parameter("hacc_rad").as_double();
  max_align_step_ = this->get_parameter("max_align_step").as_double();
  max_descent_step_ = this->get_parameter("max_descent_step").as_double();
  servo_gain_high_ = this->get_parameter("servo_gain_high").as_double();
  servo_gain_low_ = this->get_parameter("servo_gain_low").as_double();

  mpc_land_alt1_ = this->get_parameter("mpc_land_alt1").as_double();
  mpc_land_alt2_ = this->get_parameter("mpc_land_alt2").as_double();
  mpc_land_alt_crawl_ = this->get_parameter("mpc_land_alt_crawl").as_double();
  mpc_z_vel_max_dn_ = this->get_parameter("mpc_z_vel_max_dn").as_double();
  mpc_land_speed_ = this->get_parameter("mpc_land_speed").as_double();
  mpc_land_crwl_ = this->get_parameter("mpc_land_crwl").as_double();
  high_alt_ = this->get_parameter("high_alt").as_double();
  low_alt_ = this->get_parameter("low_alt").as_double();
  alpha_high_ = this->get_parameter("alpha_high").as_double();
  alpha_low_ = this->get_parameter("alpha_low").as_double();
  filter_window_ = this->get_parameter("filter_window").as_int();

  align_r_min_ = this->get_parameter("align_r_min").as_double();
  align_r_max_ = this->get_parameter("align_r_max").as_double();
  align_r_gain_ = this->get_parameter("align_r_gain").as_double();
  align_r_bias_ = this->get_parameter("align_r_bias").as_double();
  desc_r_min_ = this->get_parameter("desc_r_min").as_double();
  desc_r_max_ = this->get_parameter("desc_r_max").as_double();
  desc_r_gain_ = this->get_parameter("desc_r_gain").as_double();
  desc_r_bias_ = this->get_parameter("desc_r_bias").as_double();
  reject_r_min_ = this->get_parameter("reject_r_min").as_double();
  reject_r_max_ = this->get_parameter("reject_r_max").as_double();
  reject_r_gain_ = this->get_parameter("reject_r_gain").as_double();

  target_loss_grace_ = this->get_parameter("target_loss_grace").as_double();
  descent_loss_hold_ = this->get_parameter("descent_loss_hold").as_double();
  low_alt_max_err_ = this->get_parameter("low_alt_max_err").as_double();
  search_alt_ = this->get_parameter("search_alt").as_double();
  search_alt_max_ = this->get_parameter("search_alt_max").as_double();
  final_approach_timeout_ = this->get_parameter("final_approach_timeout").as_double();
  final_descent_rate_ = this->get_parameter("final_descent_rate").as_double();
  final_align_step_ = this->get_parameter("final_align_step").as_double();
  sp_vel_max_ = this->get_parameter("sp_vel_max").as_double();
  sp_accel_max_ = this->get_parameter("sp_accel_max").as_double();
  yaw_lock_timeout_ = this->get_parameter("yaw_lock_timeout").as_double();
  yaw_lock_min_samples_ = this->get_parameter("yaw_lock_min_samples").as_int();
  camera_mount_roll_ = this->get_parameter("camera_mount_roll").as_double();
  camera_mount_pitch_ = this->get_parameter("camera_mount_pitch").as_double();
  camera_mount_yaw_ = this->get_parameter("camera_mount_yaw").as_double();

  // --- QoS Profiles ---
  rmw_qos_profile_t pose_qos_profile = rmw_qos_profile_default;
  pose_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  pose_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  pose_qos_profile.depth = 1;
  auto pose_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(pose_qos_profile), pose_qos_profile);

  rmw_qos_profile_t state_qos_profile = rmw_qos_profile_default;
  state_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  state_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  state_qos_profile.depth = 1;
  auto state_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(state_qos_profile), state_qos_profile);

  // --- Publishers ---
  pub_sp_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
  pub_sp_raw_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
  pub_state_ = this->create_publisher<std_msgs::msg::String>("/lander/state", 10);

  // --- TF2 Initialize ---
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // --- Subscribers ---
  sub_pos_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", pose_qos,
    std::bind(&OffboardPreclandController::on_pos, this, std::placeholders::_1)
  );

  sub_target_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    target_pose_topic_, 10,
    std::bind(&OffboardPreclandController::on_target, this, std::placeholders::_1)
  );

  sub_state_ = this->create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", state_qos,
    std::bind(&OffboardPreclandController::on_state, this, std::placeholders::_1)
  );

  sub_ext_state_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
    "/mavros/extended_state", state_qos,
    std::bind(&OffboardPreclandController::on_ext_state, this, std::placeholders::_1)
  );

  sub_waypoints_ = this->create_subscription<mavros_msgs::msg::WaypointList>(
    "/mavros/mission/waypoints", state_qos,
    std::bind(&OffboardPreclandController::on_waypoints, this, std::placeholders::_1)
  );

  // --- Service Clients ---
  set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
  arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  cmd_client_ = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
  param_get_client_ = this->create_client<mavros_msgs::srv::ParamGet>("/mavros/param/get");
  param_set_client_ = this->create_client<mavros_msgs::srv::ParamSet>("/mavros/param/set");
  wp_pull_client_ = this->create_client<mavros_msgs::srv::WaypointPull>("/mavros/mission/pull");

  // --- Timers ---
  double timer_period_sec = 1.0 / ctrl_hz_;
  loop_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period_sec),
    std::bind(&OffboardPreclandController::control_loop, this)
  );

  gimbal_timer_ = this->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&OffboardPreclandController::gimbal_tick, this)
  );

  param_timer_ = this->create_wall_timer(
    std::chrono::seconds(3),
    std::bind(&OffboardPreclandController::query_px4_params, this)
  );

  RCLCPP_INFO(this->get_logger(), "OffboardPreclandController C++ ready — monitoring for AUTO.LAND");
}

// ── Callbacks ──────────────────────────────────────────────

void OffboardPreclandController::on_pos(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  pos_enu_.x = msg->pose.position.x;
  pos_enu_.y = msg->pose.position.y;
  pos_enu_.z = msg->pose.position.z;

  q_att_.w = msg->pose.orientation.w;
  q_att_.x = msg->pose.orientation.x;
  q_att_.y = msg->pose.orientation.y;
  q_att_.z = msg->pose.orientation.z;

  // Broadcast dynamic transform map -> base_link
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg->header.stamp;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  t.transform.translation.x = msg->pose.position.x;
  t.transform.translation.y = msg->pose.position.y;
  t.transform.translation.z = msg->pose.position.z;
  t.transform.rotation = msg->pose.orientation;
  tf_broadcaster_->sendTransform(t);

  double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  history_.push_back({stamp, pos_enu_, q_att_});
  if (history_.size() > 150) {
    history_.pop_front();
  }
}

void OffboardPreclandController::on_state(const mavros_msgs::msg::State::SharedPtr msg)
{
  current_mode_ = msg->mode;
  armed_ = msg->armed;
  bool was_connected = mavros_connected_;
  mavros_connected_ = msg->connected;

  if (msg->connected && !was_connected) {
    RCLCPP_INFO(this->get_logger(), "MAVROS connected — pulling waypoints...");
    pull_waypoints_immediately();
  }

  bool was_landing = is_landing_;
  is_landing_ = (msg->mode == "AUTO.LAND");

  if (is_landing_ != was_landing) {
    RCLCPP_INFO(this->get_logger(), "Landing flag: %s (mode=%s)", is_landing_ ? "true" : "false", msg->mode.c_str());
    if (is_landing_) {
      pull_waypoints_immediately();
    }
  }
}

void OffboardPreclandController::on_ext_state(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
{
  landed_state_ = msg->landed_state;
  bool was_landing = is_landing_;
  if (!is_landing_ && msg->landed_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_LANDING) {
    is_landing_ = true;
  }

  if (is_landing_ != was_landing) {
    RCLCPP_INFO(this->get_logger(), "Landing flag: %s (landed_state=%d)", is_landing_ ? "true" : "false", msg->landed_state);
    if (is_landing_) {
      pull_waypoints_immediately();
    }
  }
}

void OffboardPreclandController::on_waypoints(const mavros_msgs::msg::WaypointList::SharedPtr msg)
{
  waypoints_ = msg->waypoints;
  current_wp_seq_ = msg->current_seq;
  RCLCPP_INFO(this->get_logger(), "Received waypoints update: %d items. Active seq: %d", (int)msg->waypoints.size(), msg->current_seq);
}

void OffboardPreclandController::on_target(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  tracking_count_++;
  publish_static_transform(msg->header.frame_id);

  geometry_msgs::msg::PoseStamped msg_map;
  bool tf_ok = false;
  double world_yaw_sample = 0.0;
  double abs_x = 0.0;
  double abs_y = 0.0;
  double raw_pad_z = 0.0;
  Quaternion h_q{1.0, 0.0, 0.0, 0.0};

  try {
    geometry_msgs::msg::PoseStamped msg_zero_time = *msg;
    msg_zero_time.header.stamp = rclcpp::Time(0);
    msg_map = tf_buffer_->transform(msg_zero_time, "map", tf2::durationFromSec(0.05));
    abs_x = msg_map.pose.position.x;
    abs_y = msg_map.pose.position.y;

    raw_pad_z = msg_map.pose.position.z;

    Quaternion q_tag_world{
      msg_map.pose.orientation.w,
      msg_map.pose.orientation.x,
      msg_map.pose.orientation.y,
      msg_map.pose.orientation.z
    };
    world_yaw_sample = get_yaw(q_tag_world);
    tf_ok = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF2 Transform to map failed: %s. Using manual fallback.", ex.what());

    double tvec_x = msg->pose.position.x;
    double tvec_y = msg->pose.position.y;
    double tvec_z = msg->pose.position.z;
    double cam_x = camera_x_to_body_east_sign_ * tvec_x;
    double cam_y = camera_y_to_body_north_sign_ * tvec_y;

    double t_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    Vector3 h_pos;
    std::tie(h_pos, h_q) = get_historical_state(t_time);
    Vector3 rel = camera_to_enu(cam_x, cam_y, h_q);

    abs_x = h_pos.x + rel.x;
    abs_y = h_pos.y + rel.y;

    raw_pad_z = h_pos.z - tvec_z;

    Quaternion q_tag_cam{
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z
    };
    Quaternion q_cam_body{0.0, 0.7071067811865475, -0.7071067811865475, 0.0};
    Quaternion q_tag_world = quaternion_multiply(quaternion_multiply(h_q, q_cam_body), q_tag_cam);
    world_yaw_sample = get_yaw(q_tag_world);
  }

  // Calculate error relative to the drone's current pose
  double rel_x = abs_x - pos_enu_.x;
  double rel_y = abs_y - pos_enu_.y;
  double rn = std::sqrt(rel_x*rel_x + rel_y*rel_y);
  double rr = get_reject_r();
  if (rn > rr) {
    tracking_count_ = 0;
    return;
  }

  // Sample is valid, update virtual pad altitude with rate limit
  double max_pad_z_step = 0.02; // m/tick — giới hạn trôi tối đa
  double pad_z_target = ema_alpha_pad_ * raw_pad_z + (1.0 - ema_alpha_pad_) * virtual_pad_z_;
  double d = pad_z_target - virtual_pad_z_;
  virtual_pad_z_ += std::clamp(d, -max_pad_z_step, max_pad_z_step);

  target_enu_ = {abs_x, abs_y};
  target_rel_norm_ = rn;
  last_pose_time_ = now_sec();

  if (align_yaw_to_tag_) {
    double target_lock_alt = (yaw_lock_stage_ <= 1) ? yaw_lock_alt_ : yaw_lock_alt_2_;
    if (state_ == PrecLandState::DESCEND_ABOVE_TARGET && !yaw_locked_ && get_alt() <= target_lock_alt) {
      yaw_lock_buf_.push_back(world_yaw_sample);
      if (static_cast<int>(yaw_lock_buf_.size()) >= yaw_lock_samples_) {
        if (!yaw_lock_buf_.empty()) {
          tag_yaw_abs_ = compute_locked_yaw(yaw_lock_buf_);
          yaw_locked_ = true;
          RCLCPP_INFO(
            this->get_logger(),
            "[YAW-LOCK] latched target=%.1f deg from %d samples at %.1fm",
            tag_yaw_abs_.value() * 180.0 / M_PI, (int)yaw_lock_buf_.size(), get_alt()
          );
        }
      }
    }

    if (tracking_count_ % 15 == 0) {
      double body_yaw = tf_ok ? get_yaw(q_att_) : get_yaw(h_q);
      RCLCPP_INFO(
        this->get_logger(),
        "[YAW-3D] alt=%.1fm stage=%d body=%.1f deg | sample=%.1f deg | locked=%s target=%.1f deg | buf=%d/%d | sp=%.1f deg",
        pos_enu_.z, yaw_lock_stage_, body_yaw * 180.0 / M_PI, world_yaw_sample * 180.0 / M_PI,
        yaw_locked_ ? "true" : "false", tag_yaw_abs_.has_value() ? tag_yaw_abs_.value() * 180.0 / M_PI : 0.0,
        (int)yaw_lock_buf_.size(), yaw_lock_samples_, sp_yaw_ * 180.0 / M_PI
      );
    }
  }
}

// ── Service Helpers ───────────────────────────────────────

void OffboardPreclandController::pull_waypoints_immediately()
{
  if (wp_pull_client_->service_is_ready()) {
    auto req = std::make_shared<mavros_msgs::srv::WaypointPull::Request>();
    std::weak_ptr<OffboardPreclandController> weak_this = std::static_pointer_cast<OffboardPreclandController>(shared_from_this());
    auto cb = [weak_this](rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedFuture future) {
      auto node = weak_this.lock();
      if (!node) return;
      try {
        auto res = future.get();
        if (res->success) {
          RCLCPP_INFO(node->get_logger(), "Successfully pulled waypoints, received=%u", res->wp_received);
        } else {
          RCLCPP_WARN(node->get_logger(), "Waypoint pull failed");
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "Waypoint pull service call failed: %s", e.what());
      }
    };
    wp_pull_client_->async_send_request(req, cb);
    RCLCPP_INFO(this->get_logger(), "Landing phase started — pulling waypoints immediately");
  }
}

void OffboardPreclandController::set_mode(const std::string & mode)
{
  if (set_mode_client_->service_is_ready()) {
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;

    std::weak_ptr<OffboardPreclandController> weak_this = std::static_pointer_cast<OffboardPreclandController>(shared_from_this());
    auto cb = [weak_this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      auto node = weak_this.lock();
      if (!node) return;
      try {
        auto res = future.get();
        if (res->mode_sent) {
          RCLCPP_INFO(node->get_logger(), "Set mode %s succeeded", mode.c_str());
        } else {
          RCLCPP_WARN(node->get_logger(), "Set mode %s failed (mode_sent = false)", mode.c_str());
          if (mode == "OFFBOARD") {
            node->offboard_activated_ = false; // allow retry
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "Set mode %s service call failed: %s", mode.c_str(), e.what());
        if (mode == "OFFBOARD") {
          node->offboard_activated_ = false; // allow retry
        }
      }
    };
    set_mode_client_->async_send_request(req, cb);
  }
}

void OffboardPreclandController::send_command(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
  if (cmd_client_->service_is_ready()) {
    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->command = command;
    req->param1 = p1;
    req->param2 = p2;
    req->param3 = p3;
    req->param4 = p4;
    req->param5 = p5;
    req->param6 = p6;
    req->param7 = p7;
    cmd_client_->async_send_request(req);
  }
}

void OffboardPreclandController::disarm()
{
  RCLCPP_INFO(this->get_logger(), "Sending force-disarm (MAV_CMD 400, magic=21196)");

  if (cmd_client_->service_is_ready()) {
    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->command = 400;
    req->param1 = 0.0f;
    req->param2 = 21196.0f;

    auto cb = [this](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
      try {
        auto res = future.get();
        if (!res->success || res->result != 0 /* MAV_RESULT_ACCEPTED */) {
          RCLCPP_WARN(this->get_logger(),
            "Force-disarm command NOT accepted (success=%d, result=%d) — will retry",
            res->success, res->result);
        } else {
          RCLCPP_INFO(this->get_logger(), "Force-disarm command ACCEPTED by PX4");
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Disarm command call failed: %s", e.what());
      }
    };
    cmd_client_->async_send_request(req, cb);
  } else {
    RCLCPP_ERROR(this->get_logger(), "cmd_client_ not ready — cannot send disarm!");
  }

  // Kênh dự phòng song song qua CommandBool
  if (arm_client_->service_is_ready()) {
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = false;
    arm_client_->async_send_request(req,
      [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture f) {
        try {
          auto res = f.get();
          if (!res->success) {
            RCLCPP_WARN(this->get_logger(), "CommandBool disarm also rejected");
          }
        } catch (...) {}
      });
  }

  disarm_attempt_time_ = now_sec();
}

void OffboardPreclandController::set_px4_param_float(const std::string & param_id, float value)
{
  if (!param_set_client_->service_is_ready()) return;
  auto req = std::make_shared<mavros_msgs::srv::ParamSet::Request>();
  req->param_id = param_id;
  req->value.real = static_cast<double>(value);
  req->value.integer = 0;
  param_set_client_->async_send_request(req,
    [this, param_id, value](rclcpp::Client<mavros_msgs::srv::ParamSet>::SharedFuture f) {
      try {
        auto res = f.get();
        if (res->success) {
          RCLCPP_INFO(this->get_logger(), "Param %s set to %.2f", param_id.c_str(), value);
        } else {
          RCLCPP_WARN(this->get_logger(), "Param %s set failed", param_id.c_str());
        }
      } catch (...) {}
    });
}

void OffboardPreclandController::query_px4_params()
{
  if (param_get_client_->service_is_ready()) {
    auto req = std::make_shared<mavros_msgs::srv::ParamGet::Request>();
    req->param_id = "RTL_PLD_MD";
    using ServiceResponseFuture = rclcpp::Client<mavros_msgs::srv::ParamGet>::SharedFuture;
    std::weak_ptr<OffboardPreclandController> weak_this = std::static_pointer_cast<OffboardPreclandController>(shared_from_this());
    auto cb = [weak_this](ServiceResponseFuture future_result) {
      auto node = weak_this.lock();
      if (!node) return;
      try {
        auto res = future_result.get();
        if (res->success) {
          int val = static_cast<int>(res->value.integer);
          if (val == 0 || val == 1 || val == 2) {
            if (node->land_mode_ != val) {
              RCLCPP_INFO(node->get_logger(), "Automatically synced PX4 RTL_PLD_MD param: %d -> %d", node->land_mode_, val);
              node->land_mode_ = val;
            }
          }
        }
      } catch (const std::exception & exc) {
        RCLCPP_WARN(node->get_logger(), "Failed to query PX4 parameter RTL_PLD_MD: %s", exc.what());
      }
    };
    param_get_client_->async_send_request(req, cb);
  }
}

// ── Math Helpers ──────────────────────────────────────────

std::tuple<Vector3, Quaternion> OffboardPreclandController::get_historical_state(double time)
{
  if (history_.empty()) {
    return {pos_enu_, q_att_};
  }

  auto it = std::lower_bound(history_.begin(), history_.end(), time,
    [](const std::tuple<double, Vector3, Quaternion>& a, double val) {
      return std::get<0>(a) < val;
    }
  );

  if (it == history_.end()) {
    return {std::get<1>(history_.back()), std::get<2>(history_.back())};
  }
  if (it == history_.begin()) {
    return {std::get<1>(history_.front()), std::get<2>(history_.front())};
  }

  auto prev_it = std::prev(it);
  double diff1 = std::abs(std::get<0>(*it) - time);
  double diff2 = std::abs(std::get<0>(*prev_it) - time);

  if (diff1 < diff2) {
    return {std::get<1>(*it), std::get<2>(*it)};
  } else {
    return {std::get<1>(*prev_it), std::get<2>(*prev_it)};
  }
}

double OffboardPreclandController::get_yaw(const Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Quaternion OffboardPreclandController::quaternion_multiply(const Quaternion & q1, const Quaternion & q2)
{
  Quaternion r;
  r.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
  r.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
  r.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
  r.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
  return r;
}

double OffboardPreclandController::wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double OffboardPreclandController::circular_mean(const std::vector<double> & angles)
{
  double s = 0.0;
  double c = 0.0;
  for (double a : angles) {
    s += std::sin(a);
    c += std::cos(a);
  }
  return std::atan2(s, c);
}

double OffboardPreclandController::compute_locked_yaw(const std::vector<double> & yaw_buf)
{
  if (yaw_buf.empty()) {
    return sp_yaw_;
  }
  double avg_yaw = circular_mean(yaw_buf);
  if (tag_yaw_sign_ < 0.0) {
    avg_yaw = wrap_angle(avg_yaw + M_PI);
  }
  return wrap_angle(avg_yaw + tag_yaw_offset_);
}

void OffboardPreclandController::update_yaw()
{
  double desired = held_yaw_;
  if (align_yaw_to_tag_ && tag_yaw_abs_.has_value()) {
    desired = tag_yaw_abs_.value();
  }
  double step = yaw_slew_rate_ / ctrl_hz_;
  double err = wrap_angle(desired - sp_yaw_);
  if (std::abs(err) <= step) {
    sp_yaw_ = desired;
  } else {
    sp_yaw_ = wrap_angle(sp_yaw_ + (err > 0 ? step : -step));
  }
}

Vector3 OffboardPreclandController::camera_to_enu(double cam_x, double cam_y, const Quaternion & q_att)
{
  if (camera_yaw_frame_ == "local") {
    return Vector3{cam_x, cam_y, 0.0};
  }
  double yaw = get_yaw(q_att);
  double eb = cam_x;
  double nb = cam_y;
  double xb = nb + camera_offset_x_;
  double yb = -eb + camera_offset_y_;
  double c = std::cos(yaw);
  double s = std::sin(yaw);
  return Vector3{
    xb * c - yb * s,
    xb * s + yb * c,
    0.0
  };
}

Vector3 OffboardPreclandController::calculate_visual_setpoint(double z_sp, double max_step)
{
  auto target_val = target_enu_filtered_.has_value() ? target_enu_filtered_ : target_enu_;
  if (!target_val.has_value()) {
    return Vector3{pos_enu_.x, pos_enu_.y, z_sp};
  }
  double rel_x = std::get<0>(target_val.value()) - pos_enu_.x;
  double rel_y = std::get<1>(target_val.value()) - pos_enu_.y;
  double delta_x = get_servo_gain() * rel_x;
  double delta_y = get_servo_gain() * rel_y;
  double d = std::sqrt(delta_x*delta_x + delta_y*delta_y);
  if (d > max_step) {
    delta_x *= max_step / d;
    delta_y *= max_step / d;
  }
  return Vector3{
    pos_enu_.x + delta_x,
    pos_enu_.y + delta_y,
    z_sp
  };
}

double OffboardPreclandController::current_descent_rate()
{
  double z = get_alt();
  if (z > mpc_land_alt1_) {
    return mpc_z_vel_max_dn_;
  } else if (z > mpc_land_alt2_) {
    return mpc_land_speed_;
  } else if (z > mpc_land_alt_crawl_) {
    double span = mpc_land_alt2_ - mpc_land_alt_crawl_;
    double t = (z - mpc_land_alt_crawl_) / span;
    return mpc_land_crwl_ + (mpc_land_speed_ - mpc_land_crwl_) * t;
  } else {
    return mpc_land_crwl_;
  }
}

bool OffboardPreclandController::can_transition(PrecLandState from, PrecLandState to)
{
  if (to == PrecLandState::IDLE) return true;
  if (to == PrecLandState::DONE) return true;
  if (to == PrecLandState::FALLBACK) return true;

  switch (from) {
    case PrecLandState::IDLE:
      return (to == PrecLandState::START);
    case PrecLandState::START:
      return (to == PrecLandState::HORIZONTAL_APPROACH || to == PrecLandState::SEARCH);
    case PrecLandState::HORIZONTAL_APPROACH:
      return (to == PrecLandState::DESCEND_ABOVE_TARGET || to == PrecLandState::TARGET_LOST);
    case PrecLandState::DESCEND_ABOVE_TARGET:
      return (to == PrecLandState::FINAL_APPROACH || to == PrecLandState::TARGET_LOST || to == PrecLandState::SEARCH || to == PrecLandState::HORIZONTAL_APPROACH);
    case PrecLandState::FINAL_APPROACH:
      return false;
    case PrecLandState::SEARCH:
      return (to == PrecLandState::HORIZONTAL_APPROACH);
    case PrecLandState::TARGET_LOST:
      return (to == PrecLandState::HORIZONTAL_APPROACH || to == PrecLandState::DESCEND_ABOVE_TARGET || to == PrecLandState::SEARCH || to == PrecLandState::FINAL_APPROACH);
    case PrecLandState::FALLBACK:
      return false;
    case PrecLandState::DONE:
      return (to == PrecLandState::IDLE);
  }
  return false;
}

Vector3 OffboardPreclandController::apply_slew_rate(const Vector3 & target_sp, double dt)
{
  if (dt <= 0.0) return target_sp;

  double vx_des = (target_sp.x - sp_prev_.x) / dt;
  double vy_des = (target_sp.y - sp_prev_.y) / dt;

  double v_des_norm = std::sqrt(vx_des * vx_des + vy_des * vy_des);
  if (v_des_norm > sp_vel_max_) {
    vx_des = (vx_des / v_des_norm) * sp_vel_max_;
    vy_des = (vy_des / v_des_norm) * sp_vel_max_;
  }

  double ax = (vx_des - sp_prev_vel_.x) / dt;
  double ay = (vy_des - sp_prev_vel_.y) / dt;

  double a_norm = std::sqrt(ax * ax + ay * ay);
  if (a_norm > sp_accel_max_) {
    ax = (ax / a_norm) * sp_accel_max_;
    ay = (ay / a_norm) * sp_accel_max_;
  }

  sp_prev_vel_.x += ax * dt;
  sp_prev_vel_.y += ay * dt;

  Vector3 filtered_sp;
  filtered_sp.x = sp_prev_.x + sp_prev_vel_.x * dt;
  filtered_sp.y = sp_prev_.y + sp_prev_vel_.y * dt;
  filtered_sp.z = target_sp.z;

  sp_prev_ = filtered_sp;
  return filtered_sp;
}

void OffboardPreclandController::publish_static_transform(const std::string & camera_frame)
{
  if (tf_static_published_) {
    return;
  }
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "base_link";
  t.child_frame_id = camera_frame;
  t.transform.translation.x = camera_offset_x_;
  t.transform.translation.y = camera_offset_y_;
  t.transform.translation.z = camera_offset_z_;

  tf2::Quaternion q;
  q.setRPY(camera_mount_roll_ * M_PI / 180.0,
           camera_mount_pitch_ * M_PI / 180.0,
           camera_mount_yaw_ * M_PI / 180.0);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf_static_broadcaster_->sendTransform(t);
  tf_static_published_ = true;
  RCLCPP_INFO(this->get_logger(), "Published static transform base_link -> %s (RPY=%.1f, %.1f, %.1f)",
              camera_frame.c_str(), camera_mount_roll_, camera_mount_pitch_, camera_mount_yaw_);
}

// ── Acceptance/Rejection Gates ─────────────────────────────

double OffboardPreclandController::get_alt()
{
  return std::max(0.0, pos_enu_.z - virtual_pad_z_);
}

double OffboardPreclandController::get_blend()
{
  double span = std::max(0.1, high_alt_ - low_alt_);
  return std::min(1.0, std::max(0.0, (get_alt() - low_alt_) / span));
}

double OffboardPreclandController::get_alpha()
{
  double t = get_blend();
  return alpha_low_ * (1.0 - t) + alpha_high_ * t;
}

double OffboardPreclandController::get_servo_gain()
{
  double t = get_blend();
  return servo_gain_low_ * (1.0 - t) + servo_gain_high_ * t;
}

double OffboardPreclandController::get_align_r()
{
  double v = align_r_bias_ + align_r_gain_ * get_alt();
  return std::min(align_r_max_, std::max(align_r_min_, v));
}

double OffboardPreclandController::get_descent_r()
{
  double v = desc_r_bias_ + desc_r_gain_ * get_alt();
  return std::min(desc_r_max_, std::max(desc_r_min_, v));
}

double OffboardPreclandController::get_reject_r()
{
  double v = reject_r_gain_ * get_alt();
  return std::min(reject_r_max_, std::max(reject_r_min_, v));
}

bool OffboardPreclandController::is_target_fresh()
{
  return (now_sec() - last_pose_time_) < target_timeout_ && last_pose_time_ > 0;
}

double OffboardPreclandController::now_sec()
{
  return this->get_clock()->now().nanoseconds() * 1e-9;
}

void OffboardPreclandController::transition(PrecLandState new_state)
{
  auto to_string = [](PrecLandState s) {
    switch (s) {
      case PrecLandState::IDLE: return "IDLE";
      case PrecLandState::START: return "START";
      case PrecLandState::HORIZONTAL_APPROACH: return "HORIZONTAL_APPROACH";
      case PrecLandState::DESCEND_ABOVE_TARGET: return "DESCEND_ABOVE_TARGET";
      case PrecLandState::FINAL_APPROACH: return "FINAL_APPROACH";
      case PrecLandState::SEARCH: return "SEARCH";
      case PrecLandState::TARGET_LOST: return "TARGET_LOST";
      case PrecLandState::FALLBACK: return "FALLBACK";
      case PrecLandState::DONE: return "DONE";
    }
    return "UNKNOWN";
  };

  if (!can_transition(state_, new_state)) {
    RCLCPP_WARN(this->get_logger(), "FSM: transition from %s to %s rejected by guard", to_string(state_), to_string(new_state));
    return;
  }

  PrecLandState old = state_;
  state_ = new_state;

  RCLCPP_INFO(this->get_logger(), "FSM: %s → %s", to_string(old), to_string(new_state));

  if (new_state == PrecLandState::IDLE || new_state == PrecLandState::START) {
    yaw_locked_ = false;
    tag_yaw_abs_.reset();
    yaw_lock_buf_.clear();
    yaw_realign_complete_ = false;
    realign_cnt_ = 0;
    yaw_lock_stage_ = 0;
    target_enu_filtered_.reset();
    target_filt_vx_ = 0.0;
    target_filt_vy_ = 0.0;
  }

  if (new_state == PrecLandState::START) {
    sp_prev_ = pos_enu_;
    sp_prev_vel_ = Vector3{0.0, 0.0, 0.0};
    disarm_requested_ = false;
    auto_land_fallback_sent_ = false;
    virtual_pad_z_ = 0.0;
  }

  if (new_state == PrecLandState::FINAL_APPROACH) {
    auto target_val = target_enu_filtered_.has_value() ? target_enu_filtered_ : target_enu_;
    if (target_val.has_value()) {
      final_x_ = std::get<0>(target_val.value());
      final_y_ = std::get<1>(target_val.value());
    } else {
      // Fallback: if no target detected, keep current position
      final_x_ = pos_enu_.x;
      final_y_ = pos_enu_.y;
    }
    final_approach_start_ = now_sec();
    final_approach_entry_z_ = pos_enu_.z;  // record entry altitude for descent comparison
    sp_prev_ = pos_enu_;
    sp_prev_vel_ = Vector3{0.0, 0.0, 0.0};
  }
}

// ── Gimbal ────────────────────────────────────────────────

void OffboardPreclandController::gimbal_tick()
{
  if (!cmd_client_->service_is_ready()) {
    return;
  }
  if (!gimbal_configured_) {
    send_command(1001, 1.0f, 191.0f); // configure gimbal
    gimbal_configured_ = true;
  }
  float pitch = (state_ != PrecLandState::IDLE && state_ != PrecLandState::DONE) ? -90.0f : 0.0f;
  send_command(1000, pitch, 0.0f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), 0.0f);
  send_command(205, pitch, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f);
}

// ── Control Loop FSM ──────────────────────────────────────

void OffboardPreclandController::control_loop()
{
  double now = now_sec();
  if (last_loop_run_time_ > 0.0) {
    double dt_loop = now - last_loop_run_time_;
    if (dt_loop > 0.2) {
      RCLCPP_WARN(this->get_logger(), "Watchdog: Control loop delayed abnormally by %.3fs!", dt_loop);
      if (state_ != PrecLandState::IDLE && state_ != PrecLandState::DONE && state_ != PrecLandState::FALLBACK) {
        RCLCPP_ERROR(this->get_logger(), "Watchdog triggered: transitioning to FALLBACK");
        transition(PrecLandState::FALLBACK);
        last_loop_run_time_ = now;
        return;
      }
    }
  }
  last_loop_run_time_ = now;

  if (state_ == PrecLandState::FINAL_APPROACH) {
    if (!armed_) {
      RCLCPP_INFO(this->get_logger(), "Drone disarmed. Landing complete.");
      disarm_requested_ = false;
      transition(PrecLandState::DONE);
      return;
    }

    if (disarm_requested_ && armed_) {
      if ((now - disarm_attempt_time_) >= 0.2) {
        RCLCPP_WARN(this->get_logger(), "Retrying force-disarm (%.1fs since first attempt)",
                    now - disarm_attempt_time_first_);
        disarm();
      }

      double since_first = now - disarm_attempt_time_first_;
      if (since_first > 2.0 && !auto_land_fallback_sent_) {
        RCLCPP_ERROR(this->get_logger(),
          "Force-disarm not confirmed after 2s — escalating to AUTO.LAND as last resort");
        set_mode("AUTO.LAND");
        auto_land_fallback_sent_ = true;
      }
    }

    if (!disarm_requested_ &&
        landed_state_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND) {
      RCLCPP_INFO(this->get_logger(), "landed_state=ON_GROUND detected → force-disarm");
      disarm_requested_ = true;
      disarm_attempt_time_first_ = now_sec();
      disarm();
      // KHÔNG transition(DONE) ở đây nữa
    }
  }

  if (!is_target_fresh()) {
    target_samples_.clear();
    tracking_count_ = 0;
  }

  switch (state_) {
    case PrecLandState::IDLE:                 st_idle(); break;
    case PrecLandState::START:                st_start(); break;
    case PrecLandState::HORIZONTAL_APPROACH:   st_horizontal_approach(); break;
    case PrecLandState::DESCEND_ABOVE_TARGET:  st_descend_above_target(); break;
    case PrecLandState::FINAL_APPROACH:        st_final_approach(); break;
    case PrecLandState::SEARCH:               st_search(); break;
    case PrecLandState::TARGET_LOST:          st_target_lost(); break;
    case PrecLandState::FALLBACK:             st_fallback(); break;
    case PrecLandState::DONE:                 st_done(); break;
  }

  // Filter the target position using slew-rate limiter if we have a fresh target
  if (target_enu_.has_value() && is_target_fresh()) {
    double target_x = std::get<0>(target_enu_.value());
    double target_y = std::get<1>(target_enu_.value());
    double dt = 1.0 / ctrl_hz_;

    if (!target_enu_filtered_.has_value()) {
      sp_prev_ = Vector3{target_x, target_y, 0.0};
      sp_prev_vel_ = Vector3{0.0, 0.0, 0.0};
      target_enu_filtered_ = {target_x, target_y};
    } else {
      Vector3 filt = apply_slew_rate(Vector3{target_x, target_y, 0.0}, dt);
      target_enu_filtered_ = {filt.x, filt.y};
    }
  } else {
    target_enu_filtered_.reset();
    sp_prev_ = pos_enu_;
    sp_prev_vel_ = Vector3{0.0, 0.0, 0.0};
  }

  update_yaw();

  if (state_ != PrecLandState::IDLE && state_ != PrecLandState::DONE &&
      state_ != PrecLandState::FALLBACK) {

    // Stop publishing setpoint the moment disarm is requested
    // to avoid OFFBOARD heartbeat keeping motors alive after touch-down
    if (!(state_ == PrecLandState::FINAL_APPROACH && disarm_requested_)) {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "map";
      msg.pose.position.x = sp_enu_.x;
      msg.pose.position.y = sp_enu_.y;
      msg.pose.position.z = sp_enu_.z;
      msg.pose.orientation.z = std::sin(sp_yaw_ / 2.0);
      msg.pose.orientation.w = std::cos(sp_yaw_ / 2.0);
      pub_sp_->publish(msg);
    }
  }

  try {
    std_msgs::msg::String m;
    auto to_string = [](PrecLandState s) {
      switch (s) {
        case PrecLandState::IDLE: return "IDLE";
        case PrecLandState::START: return "START";
        case PrecLandState::HORIZONTAL_APPROACH: return "HORIZONTAL_APPROACH";
        case PrecLandState::DESCEND_ABOVE_TARGET: return "DESCEND_ABOVE_TARGET";
        case PrecLandState::FINAL_APPROACH: return "FINAL_APPROACH";
        case PrecLandState::SEARCH: return "SEARCH";
        case PrecLandState::TARGET_LOST: return "TARGET_LOST";
        case PrecLandState::FALLBACK: return "FALLBACK";
        case PrecLandState::DONE: return "DONE";
      }
      return "UNKNOWN";
    };
    m.data = to_string(state_);
    pub_state_->publish(m);
  } catch (...) {
  }
}

// ── State Handlers ────────────────────────────────────────

void OffboardPreclandController::st_idle()
{
  if (!(is_landing_ && armed_)) {
    return;
  }

  int active_mode = land_mode_;

  for (size_t idx : {static_cast<size_t>(current_wp_seq_), static_cast<size_t>(current_wp_seq_ + 1)}) {
    if (idx < waypoints_.size()) {
      const auto & wp = waypoints_[idx];
      if (wp.command == 21 || wp.command == 85) {
        active_mode = static_cast<int>(wp.param2);
        RCLCPP_INFO(
          this->get_logger(),
          "Mission landing detected: command=%d, precision land mode=%d (seq=%d)",
          wp.command, active_mode, (int)idx
        );
        break;
      }
    }
  }

  if (active_mode == 0) {
    return;
  }

  land_mode_ = active_mode;

  RCLCPP_INFO(this->get_logger(), "AUTO.LAND detected — taking over with OFFBOARD precision landing");
  land_hold_pos_ = pos_enu_;
  sp_enu_ = pos_enu_;
  held_yaw_ = get_yaw(q_att_);
  sp_yaw_ = held_yaw_;
  tag_yaw_abs_.reset();
  yaw_locked_ = false;
  yaw_lock_buf_.clear();
  start_z_sp_ = pos_enu_.z;
  approach_alt_ = pos_enu_.z;
  search_cnt_ = 0;
  offboard_activated_ = false;
  target_samples_.clear();
  target_enu_.reset();
  target_rel_norm_ = 9999.0;
  tracking_count_ = 0;
  transition(PrecLandState::START);
}

void OffboardPreclandController::st_start()
{
  Vector3 hold = land_hold_pos_.has_value() ? land_hold_pos_.value() : pos_enu_;
  double target_z = std::min(hold.z, virtual_pad_z_ + search_alt_);
  start_z_sp_ = std::max(target_z, start_z_sp_ - current_descent_rate() / ctrl_hz_);
  sp_enu_ = Vector3{hold.x, hold.y, start_z_sp_};

  if (!offboard_activated_) {
    set_mode("OFFBOARD");
    offboard_activated_ = true;
    search_start_.reset();
    RCLCPP_INFO(this->get_logger(), "Requested OFFBOARD mode");
    return;
  }

  if (current_mode_ != "OFFBOARD") {
    if (target_counter_ % ctrl_hz_ == 0) {
      set_mode("OFFBOARD");
    }
    target_counter_++;
    return;
  }

  if (is_target_fresh() && tracking_count_ >= tracking_confirm_) {
    approach_alt_ = pos_enu_.z;
    align_start_ = now_sec();
    target_counter_ = 0;
    centered_count_ = 0;
    transition(PrecLandState::HORIZONTAL_APPROACH);
    return;
  }

  if (get_alt() <= search_alt_ + 0.3) {
    if (!search_start_.has_value()) {
      search_start_ = now_sec();
      RCLCPP_INFO(this->get_logger(), "Reached search altitude (%.1fm). Waiting 5s for target acquisition...", search_alt_);
    }

    double elapsed = now_sec() - search_start_.value();
    if (elapsed > 5.0) {
      if (land_mode_ == 1) {
        RCLCPP_WARN(this->get_logger(), "Opportunistic mode: Target not found at search altitude → FALLBACK (normal landing)");
        transition(PrecLandState::FALLBACK);
      } else {
        RCLCPP_WARN(this->get_logger(), "Required mode: Target not found at search altitude → active SEARCH");
        search_start_ = now_sec();
        transition(PrecLandState::SEARCH);
      }
    }
  } else {
    search_start_.reset();
  }
}

void OffboardPreclandController::st_horizontal_approach()
{
  if (!is_target_fresh()) {
    RCLCPP_WARN(this->get_logger(), "Target lost during approach");
    target_lost_start_ = now_sec();
    target_lost_from_ = PrecLandState::HORIZONTAL_APPROACH;
    transition(PrecLandState::TARGET_LOST);
    return;
  }

  sp_enu_ = calculate_visual_setpoint(approach_alt_, max_align_step_);
  double ar = get_align_r();
  if (target_rel_norm_ <= ar) {
    centered_count_++;
  } else {
    centered_count_ = 0;
  }

  if (target_counter_ % ctrl_hz_ == 0) {
    RCLCPP_INFO(
      this->get_logger(),
      "APPROACH: alt=%.1f err=%.2f gate=%.2f cnt=%d/%d",
      get_alt(), target_rel_norm_, ar, centered_count_, align_confirm_
    );
  }
  target_counter_++;

  if (centered_count_ >= align_confirm_) {
    descent_z_sp_ = pos_enu_.z;
    target_counter_ = 0;
    centered_count_ = 0;
    descent_drift_count_ = 0;
    transition(PrecLandState::DESCEND_ABOVE_TARGET);
    return;
  }

  if (align_start_.has_value() && (now_sec() - align_start_.value()) > align_timeout_) {
    double dr = get_descent_r();
    if (target_rel_norm_ <= dr) {
      descent_z_sp_ = pos_enu_.z;
      target_counter_ = 0;
      transition(PrecLandState::DESCEND_ABOVE_TARGET);
      return;
    }
    align_start_ = now_sec();
  }
}

void OffboardPreclandController::st_descend_above_target()
{
  if (!is_target_fresh()) {
    target_lost_start_ = now_sec();
    target_lost_from_ = PrecLandState::DESCEND_ABOVE_TARGET;
    transition(PrecLandState::TARGET_LOST);
    return;
  }

  double dr = get_descent_r();
  bool descent_ok = target_rel_norm_ <= dr;

  // Low-altitude commit check
  if (get_alt() < abort_alt_param_ && !descent_ok) {
    double age = now_sec() - last_pose_time_;
    if (age <= 0.5 && target_rel_norm_ <= low_alt_max_err_) {
      RCLCPP_WARN(this->get_logger(), "Low-alt guarded commit → FINAL_APPROACH");
      transition(PrecLandState::FINAL_APPROACH);
      return;
    }
  }

  // Trigger yaw lock stage transitions
  if (align_yaw_to_tag_) {
    if (yaw_lock_stage_ == 0 && get_alt() <= yaw_lock_alt_) {
      yaw_lock_stage_ = 1;
      yaw_locked_ = false;
      yaw_lock_buf_.clear();
      yaw_realign_complete_ = false;
      realign_cnt_ = 0;
      yaw_lock_stage_start_ = now_sec();
      RCLCPP_INFO(this->get_logger(), "Entering Stage 1 Yaw Lock at 7m");
    } else if (yaw_lock_stage_ == 1 && yaw_realign_complete_ && get_alt() <= yaw_lock_alt_2_) {
      yaw_lock_stage_ = 2;
      yaw_locked_ = false;
      yaw_lock_buf_.clear();
      yaw_realign_complete_ = false;
      realign_cnt_ = 0;
      yaw_lock_stage_start_ = now_sec();
      RCLCPP_INFO(this->get_logger(), "Entering Stage 2 Yaw Lock at 3m");
    }
  }

  bool in_lock_hover = (align_yaw_to_tag_ && !yaw_realign_complete_ && (yaw_lock_stage_ == 1 || yaw_lock_stage_ == 2));

  double current_yaw_val = get_yaw(q_att_);
  double yaw_err = std::abs(wrap_angle(sp_yaw_ - current_yaw_val));
  bool rotating = (align_yaw_to_tag_ && yaw_locked_ && yaw_err > (3.0 * M_PI / 180.0));

  if (in_lock_hover) {
    double hover_z = (yaw_lock_stage_ == 1) ? yaw_lock_alt_ : yaw_lock_alt_2_;
    descent_z_sp_ = virtual_pad_z_ + hover_z;
    descent_drift_count_ = 0; // ignore drift checks while hovering

    // Check timeout
    double elapsed_hover = now_sec() - yaw_lock_stage_start_;
    if (elapsed_hover > yaw_lock_timeout_ && !yaw_locked_ && !yaw_realign_complete_) {
      if (yaw_lock_buf_.size() >= static_cast<size_t>(yaw_lock_min_samples_)) {
        tag_yaw_abs_ = compute_locked_yaw(yaw_lock_buf_);
        yaw_locked_ = true;
        RCLCPP_WARN(this->get_logger(),
          "[YAW-TIMEOUT] Stage %d timed out (%.1fs). Using circular mean of %zu samples: sp_yaw=%.1f deg",
          yaw_lock_stage_, elapsed_hover, yaw_lock_buf_.size(), tag_yaw_abs_.value() * 180.0 / M_PI
        );
      } else {
        yaw_realign_complete_ = true; // skip yaw lock stage, move on
        RCLCPP_WARN(this->get_logger(),
          "[YAW-TIMEOUT] Stage %d timed out (%.1fs) with insufficient samples (%zu/%d). Skipping yaw realign.",
          yaw_lock_stage_, elapsed_hover, yaw_lock_buf_.size(), yaw_lock_min_samples_
        );
      }
    }

    if (!yaw_locked_) {
      if (target_counter_ % 15 == 0) {
        RCLCPP_INFO(
          this->get_logger(),
          "YAW-SAMPLING [Stage %d] at %.1fm: %d/%d samples",
          yaw_lock_stage_, get_alt(), (int)yaw_lock_buf_.size(), yaw_lock_samples_
        );
      }
    } else if (rotating) {
      if (target_counter_ % 15 == 0) {
        RCLCPP_INFO(
          this->get_logger(),
          "YAW-ALIGN (ROTATING) [Stage %d]: err=%.1f deg — holding XY",
          yaw_lock_stage_, yaw_err * 180.0 / M_PI
        );
      }
    } else {
      bool xy_centered = target_rel_norm_ <= dr;
      if (xy_centered) {
        realign_cnt_++;
        if (realign_cnt_ >= 15) {
          yaw_realign_complete_ = true;
          RCLCPP_INFO(this->get_logger(), "YAW-ALIGN & RE-CENTERING COMPLETE [Stage %d] — continuing descent", yaw_lock_stage_);
        }
      } else {
        realign_cnt_ = 0;
      }

      if (target_counter_ % 15 == 0) {
        RCLCPP_INFO(
          this->get_logger(),
          "YAW-ALIGN (RE-CENTERING) [Stage %d]: err=%.2fm (gate=%.2fm), stable_cnt=%d/15",
          yaw_lock_stage_, target_rel_norm_, dr, realign_cnt_
        );
      }
    }
  } else {
    if (descent_ok) {
      descent_drift_count_ = 0;
      double abs_final_alt = virtual_pad_z_ + final_alt_param_;
      descent_z_sp_ = std::max(abs_final_alt, descent_z_sp_ - current_descent_rate() / ctrl_hz_);
    } else {
      descent_drift_count_++;
      descent_z_sp_ = pos_enu_.z;
      if (target_counter_ % ctrl_hz_ == 0) {
        RCLCPP_WARN(this->get_logger(), "DESCENT Z-LOCK: err=%.2f > gate=%.2f", target_rel_norm_, dr);
      }
      if (descent_drift_count_ >= align_confirm_) {
        bool physically_low = pos_enu_.z < (final_alt_param_ + 0.5);
        if (get_alt() > abort_alt_param_ && !physically_low) {
          search_start_ = now_sec();
          transition(PrecLandState::SEARCH);
        } else {
          align_start_ = now_sec();
          centered_count_ = 0;
          transition(PrecLandState::HORIZONTAL_APPROACH);
        }
        return;
      }
    }
  }

  if (rotating) {
    sp_enu_.z = descent_z_sp_;
  } else {
    sp_enu_ = calculate_visual_setpoint(descent_z_sp_, max_descent_step_);
  }

  if (target_counter_ % ctrl_hz_ == 0) {
    std::string phase = descent_ok ? "descending" : "z-locked";
    RCLCPP_INFO(
      this->get_logger(),
      "DESCEND (%s): alt=%.2f z_sp=%.2f rate=%.2f err=%.2f gate=%.2f",
      phase.c_str(), get_alt(), descent_z_sp_, current_descent_rate(), target_rel_norm_, dr
    );
  }
  target_counter_++;

  if (get_alt() <= final_alt_param_ + 0.15 ||
      landed_state_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND) {
    RCLCPP_INFO(this->get_logger(), "Final altitude or ground contact reached (relative_alt=%.2fm, landed=%d)",
                get_alt(), landed_state_);
    transition(PrecLandState::FINAL_APPROACH);
  }
}

void OffboardPreclandController::st_final_approach()
{
  double elapsed       = now_sec() - final_approach_start_;
  double actual_drop   = final_approach_entry_z_ - pos_enu_.z;
  double expected_drop = final_descent_rate_ * elapsed;

  // Diagnostic log every second
  if (target_counter_ % ctrl_hz_ == 0) {
    RCLCPP_INFO(this->get_logger(),
      "FINAL_APPROACH: t=%.1fs alt=%.3fm drop=%.3f/%.3fm final_xy=(%.2f,%.2f) landed=%d disarm_req=%s",
      elapsed, pos_enu_.z, actual_drop, expected_drop, final_x_, final_y_,
      (int)landed_state_, disarm_requested_ ? "true" : "false");
  }
  target_counter_++;

  if (disarm_requested_) {
    // Đã phát hiện chạm đất — ngừng hoàn toàn mọi điều chỉnh XY/Z, tránh dao động
    // trong lúc chờ PX4 xác nhận disarm.
    sp_enu_.z = pos_enu_.z - 0.2;
    return;  // waiting for PX4 to confirm disarm
  }

  // Fast-track: Disarm immediately if PX4's internal land detector confirms we are on the ground
  if (landed_state_ == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND) {
    RCLCPP_INFO(this->get_logger(), "Ground contact detected via LandedState → force-disarm");
    set_px4_param_float("COM_DISARM_LAND", 0.1f);
    disarm_requested_ = true;
    disarm_attempt_time_first_ = now_sec();
    disarm();
    return;
  }

  // --- Tiếp tục bám target, có giới hạn tốc độ chỉnh (rate-limited), chỉ khi:
  //   - target vẫn "fresh" (chưa timeout)
  //   - chưa phát hiện ground contact (đảm bảo ở nhánh trên rồi)
  if (is_target_fresh()) {
    auto target_val = target_enu_filtered_.has_value() ? target_enu_filtered_ : target_enu_;
    if (target_val.has_value()) {
      double tx = std::get<0>(target_val.value());
      double ty = std::get<1>(target_val.value());
      double dx = tx - final_x_;
      double dy = ty - final_y_;
      double dist = std::sqrt(dx*dx + dy*dy);
      if (dist > final_align_step_) {
        double scale = final_align_step_ / dist;
        final_x_ += dx * scale;
        final_y_ += dy * scale;
      } else {
        final_x_ = tx;
        final_y_ = ty;
      }
    }
    // Nếu mất target tạm thời (không fresh), giữ nguyên final_x_/final_y_ hiện tại
    // — chính là hành vi "blind descent" cũ, dùng làm fallback tự nhiên.
  }

  // Cập nhật sp_enu_ để dùng chung pub_sp_ với các state khác
  sp_enu_.x = final_x_;
  sp_enu_.y = final_y_;
  // Push setpoint xuống để ép hạ độ cao (blind theo thời gian, như cũ)
  sp_enu_.z = final_approach_entry_z_ - expected_drop;

  // Ground contact fallback: actual descent has fallen behind expected descent by > 20cm.
  if (elapsed >= 1.0 && (expected_drop - actual_drop) > 0.20) {
    RCLCPP_INFO(this->get_logger(),
      "Ground contact: blocked by %.1fcm → force-disarm (retry loop takes over)",
      (expected_drop - actual_drop) * 100.0);
    set_px4_param_float("COM_DISARM_LAND", 0.1f);
    disarm_requested_ = true;
    disarm_attempt_time_first_ = now_sec();
    disarm();
    return;
  }

  // Timeout fallback
  if (elapsed > final_approach_timeout_) {
    RCLCPP_WARN(this->get_logger(), "FINAL_APPROACH timeout (%.1fs) → force-disarm", elapsed);
    set_px4_param_float("COM_DISARM_LAND", 0.1f);
    disarm_requested_ = true;
    disarm_attempt_time_first_ = now_sec();
    disarm();
    return;
  }
}

void OffboardPreclandController::st_search()
{
  double s_alt = std::min(search_alt_, search_alt_max_);
  Vector3 anchor;
  auto target_val = target_enu_filtered_.has_value() ? target_enu_filtered_ : target_enu_;
  if (target_val.has_value()) {
    anchor.x = std::get<0>(target_val.value());
    anchor.y = std::get<1>(target_val.value());
  } else if (land_hold_pos_.has_value()) {
    anchor = land_hold_pos_.value();
  } else {
    anchor = pos_enu_;
  }

  sp_enu_ = Vector3{anchor.x, anchor.y, virtual_pad_z_ + s_alt};
  search_cnt_++;

  if (is_target_fresh() && tracking_count_ >= tracking_confirm_) {
    approach_alt_ = pos_enu_.z;
    align_start_ = now_sec();
    target_counter_ = 0;
    centered_count_ = 0;
    transition(PrecLandState::HORIZONTAL_APPROACH);
    return;
  }

  if (search_start_.has_value() && (now_sec() - search_start_.value()) > search_timeout_) {
    RCLCPP_WARN(this->get_logger(), "Search timeout");
    if (search_cnt_ >= max_search_) {
      transition(PrecLandState::FALLBACK);
    } else {
      search_start_ = now_sec();
    }
  }
}

void OffboardPreclandController::st_target_lost()
{
  if (!target_lost_start_.has_value()) {
    target_lost_start_ = now_sec();
  }

  if (is_target_fresh() && tracking_count_ >= tracking_confirm_) {
    PrecLandState resume = target_lost_from_;
    RCLCPP_INFO(this->get_logger(), "Target reacquired → resuming");
    target_lost_start_.reset();
    target_counter_ = 0;
    centered_count_ = 0;
    if (resume == PrecLandState::HORIZONTAL_APPROACH) {
      align_start_ = now_sec();
    }
    transition(resume);
    return;
  }

  double elapsed = now_sec() - target_lost_start_.value();
  sp_enu_ = pos_enu_; // Hold current position

  if (elapsed > target_loss_grace_) {
    bool physically_low = pos_enu_.z < (final_alt_param_ + 0.5);
    if (get_alt() > abort_alt_param_ && !physically_low) {
      search_start_ = now_sec();
      transition(PrecLandState::SEARCH);
    } else {
      RCLCPP_WARN(this->get_logger(), "Target lost near ground → FINAL_APPROACH");
      transition(PrecLandState::FINAL_APPROACH);
    }
  }
}

void OffboardPreclandController::st_fallback()
{
  RCLCPP_WARN(this->get_logger(), "Fallback → reverting to AUTO.LAND (GPS landing)");
  set_mode("AUTO.LAND");
  transition(PrecLandState::DONE);
}

void OffboardPreclandController::st_done()
{
  if (!armed_) {
    RCLCPP_INFO(this->get_logger(), "LANDING COMPLETE — disarmed");
    transition(PrecLandState::IDLE);
  }
}

}  // namespace precision_landing

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(precision_landing::OffboardPreclandController)
