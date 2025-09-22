#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
    MapMemoryNode();

private:
    // Subscribers and publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Map data
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool costmap_received_ = false;
    bool should_update_map_ = false;

    // Robot position
    double last_x_;
    double last_y_;
    const double distance_threshold_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // Map parameters
    const int width_ = 400;
    const int height_ = 400;
    const double resolution_ = 0.1;
    const double origin_x_ = -20.0;
    const double origin_y_ = -20.0;
    const double origin_z_ = 0.0;
    const double origin_orientation_x_ = 0.0;
    const double origin_orientation_y_ = 0.0;
    const double origin_orientation_z_ = 0.0;
    const double origin_orientation_w_ = 1.0;
    geometry_msgs::msg::Pose origin_;
    const int unknown_cost_ = 0;

    // Callbacks
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();

    // Internal helpers
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap,
                          const geometry_msgs::msg::TransformStamped::SharedPtr transform);
    void initializeGlobalMap();
};

#endif
