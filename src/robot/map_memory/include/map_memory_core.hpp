#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    void initialize(int width, int height, double resolution, geometry_msgs::msg::Pose origin);
    void updateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    nav_msgs::msg::OccupancyGrid getMap();

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
};

}  

#endif
