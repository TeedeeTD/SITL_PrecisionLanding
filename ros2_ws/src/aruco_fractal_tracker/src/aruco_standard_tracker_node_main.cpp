#include "aruco_fractal_tracker/aruco_standard_tracker_node.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<standard_tracker::ArucoStandardTracker>(options));
  rclcpp::shutdown();

  return 0;
}
