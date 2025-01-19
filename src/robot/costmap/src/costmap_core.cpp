#include <algorithm>
#include <queue>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : costmap_data_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

void CostmapCore::initialize(int width, int height, double resolution, geometry_msgs::msg::Pose origin, double inflation_radius) {
  costmap_data_->info.width = width;
  costmap_data_->info.height = height;
  costmap_data_->info.resolution = resolution;
  costmap_data_->info.origin = origin;
  costmap_data_->data.assign(width * height, -1);
  inflation_radius_ = inflation_radius;
}

void CostmapCore::update(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0);

  for (int i = 0; i < msg->ranges.size(); ++i) {
    double x = msg->ranges[i] * std::cos(msg->angle_min + i * msg->angle_increment);
    double y = msg->ranges[i] * std::sin(msg->angle_min + i * msg->angle_increment);

    int map_x = (x - costmap_data_->info.origin.position.x) / costmap_data_->info.resolution;
    int map_y = (y - costmap_data_->info.origin.position.y) / costmap_data_->info.resolution;

    if (map_x >= 0 && map_x < costmap_data_->info.width && map_y >= 0 && map_y < costmap_data_->info.height) {
      int index = map_y * costmap_data_->info.width + map_x;
      costmap_data_->data[index] = 100;
      inflateObstacle(map_x, map_y);
    }
  }
}

// THIS FUNCTION WAS COPIED FROM THE ANSWER IN THE INTEREST OF TIME
void CostmapCore::inflateObstacle(int x, int y) {
  std::queue<std::pair<int, int>> queue;
  queue.emplace(x, y);

  std::vector<std::vector<bool>> visited(costmap_data_->info.width, std::vector<bool>(costmap_data_->info.height, false));
  visited[x][y] = true;

  while (!queue.empty()) {
    int current_x = queue.front().first;
    int current_y = queue.front().second;
    queue.pop();

    // Iterate over neighboring cells
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;  // Skip the center cell

        int nx = current_x + dx;
        int ny = current_y + dy;

        // Ensure the neighbor cell is within bounds
        if (nx >= 0 && nx < static_cast<int>(costmap_data_->info.width) &&
          ny >= 0 && ny < static_cast<int>(costmap_data_->info.height) &&
          !visited[nx][ny]) {
          // Calculate the distance to the original obstacle cell
          double distance = std::hypot(nx - x, ny - y) * costmap_data_->info.resolution;

          // If within inflation radius, mark as inflated and add to queue
          if (distance <= inflation_radius_) {
              int index = ny * costmap_data_->info.width + nx;
              if (costmap_data_->data[index] < (1 - (distance / inflation_radius_)) * 100) {
                costmap_data_->data[index] = (1 - (distance / inflation_radius_)) * 100;
              }
              queue.emplace(nx, ny);
          }

          visited[nx][ny] = true;
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmap() {
  return costmap_data_;
}

} 