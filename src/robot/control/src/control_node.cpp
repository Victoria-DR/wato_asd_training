#include <memory>
#include <chrono>

#include "control_node.hpp"

ControlNode::ControlNode(int delay_ms): Node("control"), control_(robot::ControlCore())
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(delay_ms), std::bind(&ControlNode::timer_callback, this));
}

void ControlNode::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "log time");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>(2000));
  rclcpp::shutdown();
  return 0;
}
