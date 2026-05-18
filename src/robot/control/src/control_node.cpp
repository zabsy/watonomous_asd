#include <cmath>
#include <limits>
#include <optional>// get yaw from quaternion

#include "control_node.hpp"

ControlNode::ControlNode()
: Node("control"), control_(robot::ControlCore(this->get_logger())),
  lookahead_distance_(1.0),  
  linear_speed_(0.4),        
  goal_tolerance_(0.3)       
{
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // run the control loop at 10 Hz
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_) {
    return;  
  }
  if (current_path_->poses.empty()) {
    return;
  }

  double rx = robot_odom_->pose.pose.position.x;
  double ry = robot_odom_->pose.pose.position.y;

  
  auto& q = robot_odom_->pose.pose.orientation;
  double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // stop if we're close enough to the last waypoint
  const auto& last = current_path_->poses.back();
  double gdx = last.pose.position.x - rx;
  double gdy = last.pose.position.y - ry;
  if (std::sqrt(gdx * gdx + gdy * gdy) < goal_tolerance_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());  // send zeros to stop it
    return;
  }

  // find which waypoint we're currently closest to
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < current_path_->poses.size(); ++i) {
    double dx = current_path_->poses[i].pose.position.x - rx;
    double dy = current_path_->poses[i].pose.position.y - ry;
    double d = std::sqrt(dx * dx + dy * dy);
    if (d < min_dist) {
      min_dist = d;
      closest_idx = i;
    }
  }

  // from the closest waypoint forward, pick first one thats >= lookahead_distance
  std::optional<geometry_msgs::msg::Point> lookahead;
  for (size_t i = closest_idx; i < current_path_->poses.size(); ++i) {
    double dx = current_path_->poses[i].pose.position.x - rx;
    double dy = current_path_->poses[i].pose.position.y - ry;
    if (std::sqrt(dx * dx + dy * dy) >= lookahead_distance_) {
      lookahead = current_path_->poses[i].pose.position;
      break;
    }
  }

  // if all remaining waypoints are closer than the lookahead, just aim at last one
  if (!lookahead) {
    lookahead = last.pose.position;
  }

  // compute the angle to the lookahead point relative to the robots heading
  double dx = lookahead->x - rx;
  double dy = lookahead->y - ry;
  double alpha = std::atan2(dy, dx) - yaw;

  // wrap to [-pi, pi]
  while (alpha > M_PI) alpha -= 2.0 * M_PI;
  while (alpha < -M_PI) alpha += 2.0 * M_PI;

  // pure pursuit curvature 
  double dist = std::sqrt(dx * dx + dy * dy);
  double curvature = 2.0 * std::sin(alpha) / std::max(dist, 0.001);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_speed_;
  cmd.angular.z = linear_speed_ * curvature;
  cmd_vel_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
