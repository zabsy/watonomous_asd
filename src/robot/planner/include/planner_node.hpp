#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "planner_core.hpp"


// 2D grid index
struct CellIndex {
  int x;
  int y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
  bool operator==(const CellIndex& other) const { return (x == other.x && y == other.y); }
  bool operator!=(const CellIndex& other) const { return (x != other.x || y != other.y); }
};

// hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash {
  std::size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// structure representing a node in the A* open set
struct AStarNode {
  CellIndex index;
  double f_score;  // f = g + h
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// comparator for the priority queue 
struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) {
    return a.f_score > b.f_score;
  }
};

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  robot::PlannerCore planner_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PointStamped goal_;
  double robot_x_;
  double robot_y_;
  bool goal_received_;
  bool map_received_;
  bool odom_received_;  // fix against planning from origin before first odom arrives

  enum class State { WAITING_FOR_GOAL, NAVIGATING };
  State state_;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();
  void planPath();
  bool goalReached();
};

#endif
