#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "precision_landing/offboard_precland_controller.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<precision_landing::OffboardPreclandController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
