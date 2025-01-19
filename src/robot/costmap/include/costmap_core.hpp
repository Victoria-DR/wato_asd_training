#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);
    void initialize(int width, int height, double resolution, geometry_msgs::msg::Pose origin, double inflation_radius);
    void update(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void inflateObstacle(int x, int y);
    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmap();

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
    double inflation_radius_;
};

}  

#endif
