#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>

class CostmapNode : public rclcpp::Node {
public:
    CostmapNode();

private:
    // ROS2 publisher/subscriber
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    // Costmap parameters
    static constexpr int width_ = 100;
    static constexpr int height_ = 100;
    static constexpr double resolution_ = 0.1;
    static constexpr double origin_x_ = -10.0;
    static constexpr double origin_y_ = -10.0;
    static constexpr double inflation_radius_ = 1.0;
    static constexpr int max_cost_ = 100;

    // Internal occupancy grid
    int occupancy_grid_[height_][width_];

    // Methods
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void initializeCostmap();
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap(const rclcpp::Time &stamp);
};

#endif
