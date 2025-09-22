#include "map_memory_node.hpp"
#include <cmath>

MapMemoryNode::MapMemoryNode()
: Node("map_memory"),
  last_x_(0.0),
  last_y_(0.0),
  distance_threshold_(1.0),
  tf_buffer_(this->get_clock()),
  tf_listener_(std::make_unique<tf2_ros::TransformListener>(tf_buffer_))
{
    // Subscribers
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    // Publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), std::bind(&MapMemoryNode::updateMap, this));

    // Initialize global map
    origin_.position.x = origin_x_;
    origin_.position.y = origin_y_;
    origin_.position.z = origin_z_;
    origin_.orientation.x = origin_orientation_x_;
    origin_.orientation.y = origin_orientation_y_;
    origin_.orientation.z = origin_orientation_z_;
    origin_.orientation.w = origin_orientation_w_;
    initializeGlobalMap();

    RCLCPP_INFO(this->get_logger(), "MapMemoryNode initialized.");
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_costmap_ = *msg;
    costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double dist = std::hypot(x - last_x_, y - last_y_);
    if (dist >= distance_threshold_) {
        last_x_ = x;
        last_y_ = y;
        should_update_map_ = true;
    }
}

void MapMemoryNode::updateMap() {
    if (!costmap_received_ || !should_update_map_) return;

    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
            "sim_world", "robot/chassis/lidar", latest_costmap_.header.stamp, rclcpp::Duration::from_seconds(0.1));
        integrateCostmap(std::make_shared<nav_msgs::msg::OccupancyGrid>(latest_costmap_),
                         std::make_shared<geometry_msgs::msg::TransformStamped>(transform));
        map_pub_->publish(global_map_);
        should_update_map_ = false;
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
    }
}

void MapMemoryNode::initializeGlobalMap() {
    global_map_.header.frame_id = "sim_world";
    global_map_.info.width = width_;
    global_map_.info.height = height_;
    global_map_.info.resolution = resolution_;
    global_map_.info.origin = origin_;
    global_map_.data = std::vector<int8_t>(width_ * height_, unknown_cost_);
}

void MapMemoryNode::integrateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap,
                                     const geometry_msgs::msg::TransformStamped::SharedPtr transform) 
{
    if (!costmap || !transform) return;

    for (size_t i = 0; i < costmap->data.size(); ++i) {
        int local_x_idx = i % costmap->info.width;
        int local_y_idx = i / costmap->info.width;

        double local_x = local_x_idx * costmap->info.resolution + costmap->info.origin.position.x;
        double local_y = local_y_idx * costmap->info.resolution + costmap->info.origin.position.y;

        geometry_msgs::msg::PointStamped local_point;
        local_point.header = costmap->header;
        local_point.point.x = local_x;
        local_point.point.y = local_y;
        local_point.point.z = 0.0;

        geometry_msgs::msg::PointStamped global_point;
        tf2::doTransform(local_point, global_point, *transform);

        int gx = static_cast<int>((global_point.point.x - global_map_.info.origin.position.x) / global_map_.info.resolution);
        int gy = static_cast<int>((global_point.point.y - global_map_.info.origin.position.y) / global_map_.info.resolution);

        if (gx >= 0 && gx < static_cast<int>(global_map_.info.width) &&
            gy >= 0 && gy < static_cast<int>(global_map_.info.height)) 
        {
            int idx_global = gy * global_map_.info.width + gx;
            int8_t value = costmap->data[i];
            if (value > global_map_.data[idx_global]) {
                global_map_.data[idx_global] = value;
            }
        }
    }

    global_map_.header.stamp = costmap->header.stamp;
    global_map_.info.map_load_time = costmap->header.stamp;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}
