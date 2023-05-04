#include <rclcpp/rclcpp.hpp>
#include "ros2-dynamic-reconfigure/talker.hpp"

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);

  auto n = std::make_shared<rclcpp::Node>("dr_talker_node");
  auto node = dynamic_reconfigure::Talker(n);
  rclcpp::spin(n);

  return 0;
}