#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "precision_landing/rtsp_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<precision_landing::RtspPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
