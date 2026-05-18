#include <cmath>

#include "map_memory_node.hpp"

// global map covers a 100m x 100m area at 0.1m/cell
static constexpr double MAP_RESOLUTION = 0.1;
static constexpr int MAP_WIDTH = 1000;
static constexpr int MAP_HEIGHT = 1000;
static constexpr double MAP_ORIGIN_X = -50.0;
static constexpr double MAP_ORIGIN_Y = -50.0;

MapMemoryNode::MapMemoryNode()
: Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())),
  robot_x_(0.0), robot_y_(0.0), robot_yaw_(0.0),
  last_update_x_(0.0), last_update_y_(0.0),
  costmap_received_(false)
{
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

  // set up the global map dimensions and fill everything as whats unknown (-1)
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = MAP_RESOLUTION;
  global_map_.info.width = MAP_WIDTH;
  global_map_.info.height = MAP_HEIGHT;
  global_map_.info.origin.position.x = MAP_ORIGIN_X;
  global_map_.info.origin.position.y = MAP_ORIGIN_Y;
  global_map_.info.origin.position.z = 0.0;
  global_map_.data.assign(MAP_WIDTH * MAP_HEIGHT, -1);

  // publish right away so the planner has something 
  global_map_.header.stamp = this->get_clock()->now();
  map_pub_->publish(global_map_);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  
  auto& q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void MapMemoryNode::updateMap() {
  // update every timer tick to avoid getting stuck from updating by movement
  if (!costmap_received_) {
    return;
  }

  int cm_w = latest_costmap_.info.width;
  int cm_h = latest_costmap_.info.height;
  double cm_res = latest_costmap_.info.resolution;
  double cm_ox = latest_costmap_.info.origin.position.x;
  double cm_oy = latest_costmap_.info.origin.position.y;

  // how many map cells one costmap cell covers 
  int spread = std::max(1, static_cast<int>(std::round(cm_res / MAP_RESOLUTION)));

  for (int r = 0; r < cm_h; ++r) {
    for (int c = 0; c < cm_w; ++c) {
      int8_t cell_val = latest_costmap_.data[r * cm_w + c];
      if (cell_val < 0) {
        continue;  // skip unknown costmap cells
      }

      // center of this costmap cell in the robots local frame
      double local_x = cm_ox + (c + 0.5) * cm_res;
      double local_y = cm_oy + (r + 0.5) * cm_res;

      // rotate and move into the world frame using the robots current pose
      double world_x = robot_x_ + local_x * std::cos(robot_yaw_) - local_y * std::sin(robot_yaw_);
      double world_y = robot_y_ + local_x * std::sin(robot_yaw_) + local_y * std::cos(robot_yaw_);

      // top left corner of the block this costmap cell maps to on the global grid
      int mx = static_cast<int>((world_x - MAP_ORIGIN_X) / MAP_RESOLUTION);
      int my = static_cast<int>((world_y - MAP_ORIGIN_Y) / MAP_RESOLUTION);

      // fill all map cells that this costmap cell overlaps to avoid holes
      for (int dr = 0; dr < spread; ++dr) {
        for (int dc = 0; dc < spread; ++dc) {
          int nx = mx + dc;
          int ny = my + dr;
          if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT) {
            continue;
          }
          global_map_.data[ny * MAP_WIDTH + nx] = cell_val;
        }
      }
    }
  }

  global_map_.header.stamp = this->get_clock()->now();
  map_pub_->publish(global_map_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
