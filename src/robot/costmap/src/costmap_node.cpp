#include <cmath>
#include <vector>

#include "costmap_node.hpp"

// costmap is robot-centric: 40m x 40m centered on the robot

static constexpr double RESOLUTION = 0.1;        
static constexpr int WIDTH = 360;                
static constexpr int HEIGHT = 360;               
static constexpr double INFLATION_RADIUS = 1.35;  
static constexpr int MAX_COST = 100;

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // robot sits at the center of the grid
  double origin_x = -(WIDTH / 2.0) * RESOLUTION;
  double origin_y = -(HEIGHT / 2.0) * RESOLUTION;

  // start with all cells free
  std::vector<int8_t> grid(WIDTH * HEIGHT, 0);

  // step 1: convert each laser ray to a grid cell and mark it as occupied
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    if (range < scan->range_min || range > scan->range_max) {
      continue;  // skip invalid readings
    }
    double angle = scan->angle_min + i * scan->angle_increment;
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    int gx = static_cast<int>((x - origin_x) / RESOLUTION);
    int gy = static_cast<int>((y - origin_y) / RESOLUTION);

    if (gx >= 0 && gx < WIDTH && gy >= 0 && gy < HEIGHT) {
      grid[gy * WIDTH + gx] = MAX_COST;
    }
  }

  // step 2: inflate obstacles so nearby cells get a cost proportional to distance
  int inflation_cells = static_cast<int>(INFLATION_RADIUS / RESOLUTION);
  std::vector<int8_t> inflated = grid;

  for (int r = 0; r < HEIGHT; ++r) {
    for (int c = 0; c < WIDTH; ++c) {
      if (grid[r * WIDTH + c] != MAX_COST) {
        continue;  // only spread from actual obstacles
      }
      for (int dr = -inflation_cells; dr <= inflation_cells; ++dr) {
        for (int dc = -inflation_cells; dc <= inflation_cells; ++dc) {
          int nr = r + dr;
          int nc = c + dc;
          if (nr < 0 || nr >= HEIGHT || nc < 0 || nc >= WIDTH) {
            continue;
          }
          double dist = std::sqrt(static_cast<double>(dr * dr + dc * dc)) * RESOLUTION;
          if (dist > INFLATION_RADIUS) {
            continue;
          }
          int cost = static_cast<int>(MAX_COST * (1.0 - dist / INFLATION_RADIUS));
          if (cost > inflated[nr * WIDTH + nc]) {
            inflated[nr * WIDTH + nc] = static_cast<int8_t>(cost);
          }
        }
      }
    }
  }

  // step 3: put into an occupancy grid and publish it
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "base_link";
  msg.info.resolution = RESOLUTION;
  msg.info.width = WIDTH;
  msg.info.height = HEIGHT;
  msg.info.origin.position.x = origin_x;
  msg.info.origin.position.y = origin_y;
  msg.info.origin.position.z = 0.0;
  msg.data = inflated;

  costmap_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
