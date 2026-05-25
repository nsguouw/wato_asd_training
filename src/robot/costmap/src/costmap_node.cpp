#include <chrono>
#include <memory>
#include <cmath>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::initializeCostmap() {
  costmap_grid_.assign(HEIGHT, std::vector<int>(WIDTH, 0));
}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);
  x_grid = static_cast<int>((x / RESOLUTION) + WIDTH / 2);
  y_grid = static_cast<int>((y / RESOLUTION) + HEIGHT / 2);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < WIDTH && y_grid >= 0 && y_grid < HEIGHT) {
    costmap_grid_[y_grid][x_grid] = MAX_COST;
  }
}

void CostmapNode::inflateObstacles() {
  int inflation_cells = static_cast<int>(INFLATION_RADIUS / RESOLUTION);
  std::vector<std::vector<int>> inflated = costmap_grid_;

  for (int y = 0; y < HEIGHT; ++y) {
    for (int x = 0; x < WIDTH; ++x) {
      if (costmap_grid_[y][x] == MAX_COST) {
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < WIDTH && ny >= 0 && ny < HEIGHT) {
              double distance = std::sqrt(dx * dx + dy * dy) * RESOLUTION;
              if (distance <= INFLATION_RADIUS) {
                int cost = static_cast<int>(MAX_COST * (1.0 - distance / INFLATION_RADIUS));
                inflated[ny][nx] = std::max(inflated[ny][nx], cost);
              }
            }
          }
        }
      }
    }
  }
  costmap_grid_ = inflated;
}

void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "odom";
  msg.info.resolution = RESOLUTION;
  msg.info.width = WIDTH;
  msg.info.height = HEIGHT;
  msg.info.origin.position.x = -(WIDTH * RESOLUTION) / 2.0;
  msg.info.origin.position.y = -(HEIGHT * RESOLUTION) / 2.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(WIDTH * HEIGHT);
  for (int y = 0; y < HEIGHT; ++y) {
    for (int x = 0; x < WIDTH; ++x) {
      msg.data[y * WIDTH + x] = costmap_grid_[y][x];
    }
  }
  costmap_pub_->publish(msg);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  initializeCostmap();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
      int x_grid, y_grid;
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }

  inflateObstacles();
  publishCostmap();
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}