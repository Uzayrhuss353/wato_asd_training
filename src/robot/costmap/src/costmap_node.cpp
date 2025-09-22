#include "costmap_node.hpp"
#include <cmath>
#include <algorithm>

CostmapNode::CostmapNode() : Node("costmap") {
    // Initialize occupancy grid
    initializeCostmap();

    // Create LIDAR subscriber
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10,
        std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

    // Create costmap publisher
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

    RCLCPP_INFO(this->get_logger(), "CostmapNode initialized.");
}

void CostmapNode::initializeCostmap() {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            occupancy_grid_[y][x] = 0;
        }
    }
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    initializeCostmap();

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range > scan->range_min && range < scan->range_max) {
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }

    inflateObstacles();
    publishCostmap(scan->header.stamp);
}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    x_grid = static_cast<int>((x - origin_x_) / resolution_);
    y_grid = static_cast<int>((y - origin_y_) / resolution_);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
    if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
        occupancy_grid_[y_grid][x_grid] = max_cost_;
    }
}

void CostmapNode::inflateObstacles() {
    int temp[height_][width_];
    for (int y = 0; y < height_; ++y)
        for (int x = 0; x < width_; ++x)
            temp[y][x] = occupancy_grid_[y][x];

    int radius_cells = static_cast<int>(inflation_radius_ / resolution_);

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (temp[y][x] == max_cost_) {
                for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            double dist = std::hypot(dx * resolution_, dy * resolution_);
                            if (dist <= inflation_radius_) {
                                int cost = static_cast<int>(max_cost_ * (1.0 - dist / inflation_radius_));
                                occupancy_grid_[ny][nx] = std::max(occupancy_grid_[ny][nx], cost);
                            }
                        }
                    }
                }
            }
        }
    }
}

void CostmapNode::publishCostmap(const rclcpp::Time &stamp) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.header.stamp = stamp;
    grid.info.resolution = resolution_;
    grid.info.width = width_;
    grid.info.height = height_;
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(width_ * height_);

    for (int y = 0; y < height_; ++y)
        for (int x = 0; x < width_; ++x)
            grid.data[y * width_ + x] = static_cast<int8_t>(occupancy_grid_[y][x]);

    costmap_pub_->publish(grid);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}
