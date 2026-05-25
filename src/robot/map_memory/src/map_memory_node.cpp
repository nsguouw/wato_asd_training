#include <cmath>
#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

  initializeGlobalMap();
}

void MapMemoryNode::initializeGlobalMap() {
  global_map_.header.frame_id = "odom";
  global_map_.info.resolution = RESOLUTION;
  global_map_.info.width = WIDTH;
  global_map_.info.height = HEIGHT;
  global_map_.info.origin.position.x = -(WIDTH * RESOLUTION) / 2.0;
  global_map_.info.origin.position.y = -(HEIGHT * RESOLUTION) / 2.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(WIDTH * HEIGHT, -1);
  map_initialized_ = true;
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // Extract yaw from quaternion
  auto q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  double distance = std::sqrt(std::pow(robot_x_ - last_x_, 2) +
                               std::pow(robot_y_ - last_y_, 2));
  if (distance >= DISTANCE_THRESHOLD) {
    last_x_ = robot_x_;
    last_y_ = robot_y_;
    should_update_map_ = true;
  }
}

void MapMemoryNode::integrateCostmap() {
  int costmap_width = latest_costmap_.info.width;
  int costmap_height = latest_costmap_.info.height;
  double costmap_res = latest_costmap_.info.resolution;

  for (int cy = 0; cy < costmap_height; ++cy) {
    for (int cx = 0; cx < costmap_width; ++cx) {
      int idx = cy * costmap_width + cx;
      int8_t cell_val = latest_costmap_.data[idx];
      if (cell_val < 0) continue;  // skip unknown cells

      // Local position relative to costmap origin (robot-centered)
      double local_x = (cx - costmap_width / 2.0) * costmap_res;
      double local_y = (cy - costmap_height / 2.0) * costmap_res;

      // Rotate and translate to global frame
      double global_x = robot_x_ + local_x * std::cos(robot_yaw_) - local_y * std::sin(robot_yaw_);
      double global_y = robot_y_ + local_x * std::sin(robot_yaw_) + local_y * std::cos(robot_yaw_);

      // Convert to global map indices
      int mx = static_cast<int>((global_x - global_map_.info.origin.position.x) / RESOLUTION);
      int my = static_cast<int>((global_y - global_map_.info.origin.position.y) / RESOLUTION);

      if (mx >= 0 && mx < WIDTH && my >= 0 && my < HEIGHT) {
        global_map_.data[my * WIDTH + mx] = cell_val;
      }
    }
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {
    integrateCostmap();
    global_map_.header.stamp = this->get_clock()->now();
    map_pub_->publish(global_map_);
    should_update_map_ = false;
  }
  // Always publish on init so planner has a map immediately
  if (map_initialized_) {
    global_map_.header.stamp = this->get_clock()->now();
    map_pub_->publish(global_map_);
  }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}