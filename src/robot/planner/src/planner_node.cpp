#include <cmath>
#include <algorithm>
#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  map_received_ = true;
  if (state_ == State::NAVIGATING) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::NAVIGATING;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::NAVIGATING) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    } else {
      planPath();
    }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5;
}

std::vector<CellIndex> PlannerNode::runAStar(CellIndex start, CellIndex goal) {
  int width = current_map_.info.width;
  int height = current_map_.info.height;

  auto heuristic = [&](CellIndex a, CellIndex b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
  };

  auto inBounds = [&](int x, int y) {
    return x >= 0 && x < width && y >= 0 && y < height;
  };

  auto isObstacle = [&](int x, int y) {
    int idx = y * width + x;
    return current_map_.data[idx] > 20;
  };

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(start, goal));

  std::vector<std::pair<int,int>> directions = {
    {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}
  };

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal) {
      // Reconstruct path
      std::vector<CellIndex> path;
      while (current != start) {
        path.push_back(current);
        current = came_from[current];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (auto [dx, dy] : directions) {
      int nx = current.x + dx;
      int ny = current.y + dy;
      if (!inBounds(nx, ny) || isObstacle(nx, ny)) continue;

      double move_cost = (dx != 0 && dy != 0) ? 1.414 : 1.0;
      double tentative_g = g_score[current] + move_cost;
      CellIndex neighbor(nx, ny);

      if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
        g_score[neighbor] = tentative_g;
        came_from[neighbor] = current;
        double f = tentative_g + heuristic(neighbor, goal);
        open_set.emplace(neighbor, f);
      }
    }
  }
  return {};  // No path found
}

void PlannerNode::planPath() {
  if (!goal_received_ || !map_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan: missing map or goal");
    return;
  }

  double res = current_map_.info.resolution;
  double ox = current_map_.info.origin.position.x;
  double oy = current_map_.info.origin.position.y;
  int width = current_map_.info.width;
  int height = current_map_.info.height;

  // Convert robot position to grid
  int start_x = static_cast<int>((robot_pose_.position.x - ox) / res);
  int start_y = static_cast<int>((robot_pose_.position.y - oy) / res);

  // Convert goal position to grid
  int goal_x = static_cast<int>((goal_.point.x - ox) / res);
  int goal_y = static_cast<int>((goal_.point.y - oy) / res);

  // Clamp to map bounds
  start_x = std::clamp(start_x, 0, width - 1);
  start_y = std::clamp(start_y, 0, height - 1);
  goal_x = std::clamp(goal_x, 0, width - 1);
  goal_y = std::clamp(goal_y, 0, height - 1);

  auto cell_path = runAStar(CellIndex(start_x, start_y), CellIndex(goal_x, goal_y));

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "odom";

  for (auto &cell : cell_path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = ox + (cell.x + 0.5) * res;
    pose.pose.position.y = oy + (cell.y + 0.5) * res;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  path_pub_->publish(path);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}