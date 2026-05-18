#include <cmath>
#include <algorithm>
#include <limits>

#include "planner_node.hpp"

PlannerNode::PlannerNode()
: Node("planner"), planner_(robot::PlannerCore(this->get_logger())),
  robot_x_(0.0), robot_y_(0.0),
  goal_received_(false), map_received_(false), odom_received_(false),
  state_(State::WAITING_FOR_GOAL)
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // check goal status every 500ms and replan if is needed
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  map_received_ = true;
  // replan whenever the map updates incase if can find a better route 
  if (state_ == State::NAVIGATING) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::NAVIGATING;
  RCLCPP_INFO(this->get_logger(), "got a new goal: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  odom_received_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ == State::NAVIGATING) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "made it to the goal, nice");
      state_ = State::WAITING_FOR_GOAL;
    }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_x_;
  double dy = goal_.point.y - robot_y_;
  return std::sqrt(dx * dx + dy * dy) < 0.5;
}

void PlannerNode::planPath() {
  if (!goal_received_ || !map_received_ || !odom_received_) {
    RCLCPP_WARN(this->get_logger(), "waiting for map, goal, and odom before planning");
    return;
  }

  double res = current_map_.info.resolution;
  int width = static_cast<int>(current_map_.info.width);
  int height = static_cast<int>(current_map_.info.height);
  double ox = current_map_.info.origin.position.x;
  double oy = current_map_.info.origin.position.y;

  // convert world coordinates to grid cell indices
  auto worldToCell = [&](double wx, double wy) -> CellIndex {
    return CellIndex(
      static_cast<int>((wx - ox) / res),
      static_cast<int>((wy - oy) / res));
  };

  // convert grid cell back to the center of that cell in world coordinates
  auto cellToWorld = [&](const CellIndex& c) -> std::pair<double, double> {
    return {ox + (c.x + 0.5) * res, oy + (c.y + 0.5) * res};
  };

  auto inBounds = [&](const CellIndex& c) -> bool {
    return c.x >= 0 && c.x < width && c.y >= 0 && c.y < height;
  };

  // cells with cost > 30 are treated as obstacles 
  auto isObstacle = [&](const CellIndex& c) -> bool {
    int8_t val = current_map_.data[c.y * width + c.x];
    return val > 30;
  };

  CellIndex start = worldToCell(robot_x_, robot_y_);
  CellIndex goal_cell = worldToCell(goal_.point.x, goal_.point.y);

  // euclidean distance heuristic
  auto heuristic = [&](const CellIndex& c) -> double {
    double dx = c.x - goal_cell.x;
    double dy = c.y - goal_cell.y;
    return std::sqrt(dx * dx + dy * dy);
  };

  // a* 
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_set<CellIndex, CellIndexHash> closed_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  g_score[start] = 0.0;
  open_set.push(AStarNode(start, heuristic(start)));

  
  int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

  bool found = false;
  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();

    if (closed_set.count(current.index)) {
      continue;  // already processed this cell with a better cost
    }
    closed_set.insert(current.index);

    if (current.index == goal_cell) {
      found = true;
      break;
    }

    for (int i = 0; i < 8; ++i) {
      CellIndex neighbor(current.index.x + dx[i], current.index.y + dy[i]);
      if (!inBounds(neighbor) || isObstacle(neighbor) || closed_set.count(neighbor)) {
        continue;
      }
      // diagonal moves cost sqrt(2), cardinal moves cost 1
      double step_cost = (dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0;
      double tentative_g = g_score[current.index] + step_cost;

      if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
        g_score[neighbor] = tentative_g;
        came_from[neighbor] = current.index;
        open_set.push(AStarNode(neighbor, tentative_g + heuristic(neighbor)));
      }
    }
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "sim_world";

  if (!found) {
    RCLCPP_WARN(this->get_logger(), "no path found, map might not have enough info yet");
    path_pub_->publish(path);
    return;
  }

  // trace back from goal to start and reverse it
  std::vector<CellIndex> raw_path;
  CellIndex cur = goal_cell;
  while (cur != start) {
    raw_path.push_back(cur);
    cur = came_from[cur];
  }
  raw_path.push_back(start);
  std::reverse(raw_path.begin(), raw_path.end());

  for (const auto& cell : raw_path) {
    auto world_pos = cellToWorld(cell);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = world_pos.first;
    pose.pose.position.y = world_pos.second;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  path_pub_->publish(path);
  RCLCPP_INFO(this->get_logger(), "published path with %zu waypoints", raw_path.size());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
