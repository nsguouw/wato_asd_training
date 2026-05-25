#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  robot::CostmapCore costmap_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void initializeCostmap();
  void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
  void markObstacle(int x_grid, int y_grid);
  void inflateObstacles();
  void publishCostmap();

  // Costmap parameters
  static constexpr double RESOLUTION = 0.1;  // meters per cell
  static constexpr int WIDTH = 150;           // cells (10m)
  static constexpr int HEIGHT = 150;          // cells (10m)
  static constexpr double INFLATION_RADIUS = 1.5;  // meters
  static constexpr int MAX_COST = 100;

  std::vector<std::vector<int>> costmap_grid_;
};

#endif