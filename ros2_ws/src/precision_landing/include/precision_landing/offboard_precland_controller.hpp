#ifndef PRECISION_LANDING__OFFBOARD_PRECLAND_CONTROLLER_HPP_
#define PRECISION_LANDING__OFFBOARD_PRECLAND_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <deque>
#include <tuple>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/param_get.hpp>
#include <mavros_msgs/srv/waypoint_pull.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace precision_landing
{

enum class PrecLandState
{
  IDLE,
  START,
  HORIZONTAL_APPROACH,
  DESCEND_ABOVE_TARGET,
  FINAL_APPROACH,
  SEARCH,
  TARGET_LOST,
  FALLBACK,
  DONE
};

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

class OffboardPreclandController : public rclcpp::Node
{
public:
  OffboardPreclandController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~OffboardPreclandController() = default;

private:
  // --- ROS 2 Callbacks ---
  void on_pos(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void on_state(const mavros_msgs::msg::State::SharedPtr msg);
  void on_ext_state(const mavros_msgs::msg::ExtendedState::SharedPtr msg);
  void on_waypoints(const mavros_msgs::msg::WaypointList::SharedPtr msg);
  void on_target(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // --- Main Loop and FSM Tick ---
  void control_loop();
  void gimbal_tick();
  void query_px4_params();

  // --- FSM State Handlers ---
  void st_idle();
  void st_start();
  void st_horizontal_approach();
  void st_descend_above_target();
  void st_final_approach();
  void st_search();
  void st_target_lost();
  void st_fallback();
  void st_done();

  // --- Mode and Command Helpers ---
  void set_mode(const std::string & mode);
  void send_command(uint16_t command, float p1=0.0f, float p2=0.0f, float p3=0.0f, float p4=0.0f, float p5=0.0f, float p6=0.0f, float p7=0.0f);
  void disarm();
  void pull_waypoints_immediately();
  void transition(PrecLandState new_state);
  bool can_transition(PrecLandState from, PrecLandState to);

  // --- Math and Frame Helpers ---
  std::tuple<Vector3, Quaternion> get_historical_state(double time);
  double get_yaw(const Quaternion & q);
  Quaternion quaternion_multiply(const Quaternion & q1, const Quaternion & q2);
  double wrap_angle(double angle);
  double circular_mean(const std::vector<double> & angles);
  void update_yaw();
  Vector3 camera_to_enu(double cam_x, double cam_y, const Quaternion & q_att);
  Vector3 calculate_visual_setpoint(double z_sp, double max_step);
  double current_descent_rate();
  Vector3 apply_slew_rate(const Vector3 & target_sp, double dt);
  void publish_static_transform(const std::string & camera_frame);
  double compute_locked_yaw(const std::vector<double> & yaw_buf);

  // --- Dynamic Acceptance and Rejection Gates ---
  double get_alt();
  double get_blend();
  double get_alpha();
  double get_servo_gain();
  double get_align_r();
  double get_descent_r();
  double get_reject_r();
  bool is_target_fresh();
  double now_sec();

  // --- Parameters ---
  double camera_x_to_body_east_sign_;
  double camera_y_to_body_north_sign_;
  std::string camera_yaw_frame_;
  double camera_offset_x_;
  double camera_offset_y_;
  double camera_offset_z_;
  double marker_size_;
  std::string target_topic_;
  std::string target_pose_topic_;
  bool align_yaw_to_tag_;
  double tag_yaw_sign_;
  double tag_yaw_offset_;
  int land_mode_;
  double abort_alt_param_;
  double final_alt_param_;
  double yaw_slew_rate_;
  int yaw_lock_samples_;
  double yaw_lock_alt_;
  double yaw_lock_alt_2_;

  // Tunable Controller Constants
  int ctrl_hz_;
  double target_timeout_;
  int tracking_confirm_;
  int align_confirm_;
  double align_timeout_;
  double search_timeout_;
  int max_search_;
  double descent_rate_;
  double fappr_alt_;
  double hacc_rad_;
  double max_align_step_;
  double max_descent_step_;
  double servo_gain_high_;
  double servo_gain_low_;

  double mpc_land_alt1_;
  double mpc_land_alt2_;
  double mpc_land_alt_crawl_;
  double mpc_z_vel_max_dn_;
  double mpc_land_speed_;
  double mpc_land_crwl_;
  double high_alt_;
  double low_alt_;
  double alpha_high_;
  double alpha_low_;
  int filter_window_;

  double align_r_min_;
  double align_r_max_;
  double align_r_gain_;
  double align_r_bias_;
  double desc_r_min_;
  double desc_r_max_;
  double desc_r_gain_;
  double desc_r_bias_;
  double reject_r_min_;
  double reject_r_max_;
  double reject_r_gain_;

  double target_loss_grace_;
  double descent_loss_hold_;
  double low_alt_max_err_;
  double search_alt_;
  double search_alt_max_;

  double final_approach_timeout_;
  double final_descent_rate_;
  double sp_vel_max_;
  double sp_accel_max_;
  double yaw_lock_timeout_;
  int yaw_lock_min_samples_;
  double camera_mount_roll_;
  double camera_mount_pitch_;
  double camera_mount_yaw_;

  // --- State Variables ---
  PrecLandState state_{PrecLandState::IDLE};
  Vector3 pos_enu_{0.0, 0.0, 0.0};
  Quaternion q_att_{1.0, 0.0, 0.0, 0.0};
  Vector3 sp_enu_{0.0, 0.0, 0.0};
  double sp_yaw_{0.0};
  double held_yaw_{0.0};
  std::optional<double> tag_yaw_abs_;
  bool yaw_locked_{false};
  std::vector<double> yaw_lock_buf_;
  bool yaw_realign_complete_{false};
  int realign_cnt_{0};
  int yaw_lock_stage_{0};
  double final_approach_start_{0.0};
  double yaw_lock_stage_start_{0.0};
  double last_loop_run_time_{0.0};
  double final_x_{0.0};
  double final_y_{0.0};
  Vector3 sp_prev_{0.0, 0.0, 0.0};
  Vector3 sp_prev_vel_{0.0, 0.0, 0.0};

  std::string current_mode_;
  uint8_t landed_state_{0};
  bool is_landing_{false};
  bool armed_{false};
  bool mavros_connected_{false};

  // --- Target tracking ---
  std::deque<std::tuple<double, double>> target_samples_; // Queue for filtering
  std::optional<std::tuple<double, double>> target_enu_;
  std::optional<std::tuple<double, double>> target_enu_filtered_;
  double target_filt_vx_{0.0};
  double target_filt_vy_{0.0};
  double target_rel_norm_{9999.0};
  double last_pose_time_{0.0};
  int tracking_count_{0};
  std::deque<std::tuple<double, Vector3, Quaternion>> history_;
  std::vector<mavros_msgs::msg::Waypoint> waypoints_;
  uint16_t current_wp_seq_{0};

  // --- FSM counters ---
  int centered_count_{0};
  int descent_drift_count_{0};
  double descent_z_sp_{10.0};
  int target_counter_{0};
  std::optional<double> search_start_;
  std::optional<double> align_start_;
  int search_cnt_{0};
  double approach_alt_{10.0};
  double start_z_sp_{10.0};
  std::optional<double> target_lost_start_;
  PrecLandState target_lost_from_{PrecLandState::HORIZONTAL_APPROACH};
  std::optional<Vector3> land_hold_pos_;
  bool gimbal_configured_{false};
  bool offboard_activated_{false};

  // --- Publishers ---
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_sp_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_sp_raw_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;

  // --- TF2 ---
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool tf_static_published_{false};

  // --- Subscribers ---
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pos_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_state_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr sub_ext_state_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr sub_waypoints_;

  // --- Service Clients ---
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr cmd_client_;
  rclcpp::Client<mavros_msgs::srv::ParamGet>::SharedPtr param_get_client_;
  rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedPtr wp_pull_client_;

  // --- Timers ---
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::TimerBase::SharedPtr gimbal_timer_;
  rclcpp::TimerBase::SharedPtr param_timer_;
};

}  // namespace precision_landing

#endif  // PRECISION_LANDING__OFFBOARD_PRECLAND_CONTROLLER_HPP_
