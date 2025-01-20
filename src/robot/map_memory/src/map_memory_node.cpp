#include <chrono>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  origin_.position.x = 0.0;
  origin_.position.y = 0.0;
  map_memory_.initialize(100, 100, 0.1, origin_);
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  bool all_zero = std::all_of(msg->data.begin(), msg->data.end(), [](int8_t val) { return val == 0; });

  if (!all_zero && !std::isnan(last_x_)) {
    double dist = std::hypot(x_ - last_x_, y_ - last_y_);
    if (dist >= 1.5) {
      map_memory_.updateCostmap(msg);
      last_x_ = x_;
      last_y_ = y_;
    }
  }
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
}

void MapMemoryNode::updateMap() {
  nav_msgs::msg::OccupancyGrid map_msg = map_memory_.getMap();
  map_msg.header.stamp = this->now();
  map_msg.header.frame_id = "sim_world";
  map_pub_->publish(map_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
