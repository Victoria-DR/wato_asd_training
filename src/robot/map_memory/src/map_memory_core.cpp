#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : logger_(logger) {}

void MapMemoryCore::initialize(int width, int height, double resolution, geometry_msgs::msg::Pose origin) {
  global_map_.info.width = width;
  global_map_.info.height = height;
  global_map_.info.resolution = resolution;
  global_map_.info.origin = origin;
  global_map_.data.resize(width * height, -1);
}

void MapMemoryCore::updateCostmap(nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  for (int i = 0; i < global_map_.info.height; i++) {
    for (int j = 0; j < global_map_.info.width; j++) {
      int index = i * global_map_.info.width + j;
      if (costmap->data[index] >= 0) {
        global_map_.data[index] = costmap->data[index];
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getMap() {
  return global_map_;
}

}
