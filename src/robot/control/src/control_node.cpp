
#include <cmath>
#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
    1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  if (!current_path_ || current_path_->poses.empty()) return std::nullopt;

  auto robot_pos = robot_odom_->pose.pose.position;

  // Find the first point on the path that is at least lookahead_distance_ away
  for (const auto &pose : current_path_->poses) {
    double dist = computeDistance(robot_pos, pose.pose.position);
    if (dist >= lookahead_distance_) {
      return pose;
    }
  }

  // If no point is far enough, return the last point
  return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd_vel;

  auto robot_pos = robot_odom_->pose.pose.position;
  double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

  // Check if we've reached the final goal
  if (computeDistance(robot_pos, current_path_->poses.back().pose.position) < goal_tolerance_) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return cmd_vel;
  }

  // Compute angle to lookahead point in robot frame
  double dx = target.pose.position.x - robot_pos.x;
  double dy = target.pose.position.y - robot_pos.y;
  double angle_to_target = std::atan2(dy, dx);
  double heading_error = angle_to_target - robot_yaw;

  // Normalize heading error to [-pi, pi]
  while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
  while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

  // Pure pursuit curvature
  double dist = computeDistance(robot_pos, target.pose.position);
  double curvature = 2.0 * std::sin(heading_error) / dist;

  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = linear_speed_ * curvature;

  // Clamp angular velocity
  cmd_vel.angular.z = std::max(-1.5, std::min(1.5, cmd_vel.angular.z));

  return cmd_vel;
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_) return;
  if (current_path_->poses.empty()) return;

  auto lookahead = findLookaheadPoint();
  if (!lookahead) return;

  auto cmd_vel = computeVelocity(*lookahead);
  cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}