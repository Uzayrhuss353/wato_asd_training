#include "planner_node.hpp"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

PlannerNode::PlannerNode() 
: Node("planner_node"), state_(State::WAITING_FOR_GOAL), goal_received_(false)
{
    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Timer for replanning and goal checking
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PlannerNode::timerCallback, this));
}

// ------------------- Callbacks -------------------

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    current_map_ = *msg;
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
    {
        planPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    goal_ = *msg;
    goal_received_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_pose_ = msg->pose.pose;
}

// ------------------- Timer -------------------

void PlannerNode::timerCallback()
{
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
    {
        if (goalReached())
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = State::WAITING_FOR_GOAL;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or map update...");
            planPath();
        }
    }
}

// ------------------- Helper Functions -------------------

bool PlannerNode::goalReached()
{
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.5; // threshold in meters
}

// A* utility
double PlannerNode::heuristic(const CellIndex &a, const CellIndex &b)
{
    return std::abs(a.x - b.x) + std::abs(a.y - b.y); // Manhattan distance
}

bool PlannerNode::isOccupied(const CellIndex &cell)
{
    int idx = cell.y * current_map_.info.width + cell.x;
    if (idx < 0 || idx >= static_cast<int>(current_map_.data.size()))
        return true; // outside map is obstacle
    return current_map_.data[idx] > 50;
}

// ------------------- Path Planning -------------------

void PlannerNode::planPath()
{
    if (!goal_received_ || current_map_.data.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
        return;
    }

    int width = current_map_.info.width;
    int height = current_map_.info.height;
    double res = current_map_.info.resolution;
    double origin_x = current_map_.info.origin.position.x;
    double origin_y = current_map_.info.origin.position.y;

    // Convert robot and goal positions to grid indices
    CellIndex start_idx(
        static_cast<int>((robot_pose_.position.x - origin_x) / res),
        static_cast<int>((robot_pose_.position.y - origin_y) / res));

    CellIndex goal_idx(
        static_cast<int>((goal_.point.x - origin_x) / res),
        static_cast<int>((goal_.point.y - origin_y) / res));

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;

    g_score[start_idx] = 0;
    open_set.emplace(start_idx, heuristic(start_idx, goal_idx), 0, start_idx);

    std::vector<CellIndex> directions = {{1,0},{-1,0},{0,1},{0,-1}};

    bool path_found = false;
    CellIndex last_idx = start_idx;

    while (!open_set.empty())
    {
        AStarNode current = open_set.top();
        open_set.pop();
        last_idx = current.index;

        if (current.index == goal_idx)
        {
            path_found = true;
            break;
        }

        for (auto &dir : directions)
        {
            CellIndex neighbor(current.index.x + dir.x, current.index.y + dir.y);

            if (neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= width || neighbor.y >= height)
                continue;

            if (isOccupied(neighbor))
                continue;

            double tentative_g = current.g_score + 1.0;
            if (!g_score.count(neighbor) || tentative_g < g_score[neighbor])
            {
                g_score[neighbor] = tentative_g;
                double f = tentative_g + heuristic(neighbor, goal_idx);
                open_set.emplace(neighbor, f, tentative_g, current.index);
                came_from[neighbor] = current.index;
            }
        }
    }

    // Reconstruct path
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";

    if (path_found)
    {
        CellIndex idx = goal_idx;
        std::vector<CellIndex> reversed_path;
        while (!(idx == start_idx))
        {
            reversed_path.push_back(idx);
            idx = came_from[idx];
        }
        reversed_path.push_back(start_idx);

        for (auto it = reversed_path.rbegin(); it != reversed_path.rend(); ++it)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = it->x * res + origin_x + res/2.0;
            pose.pose.position.y = it->y * res + origin_y + res/2.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "A* could not find a path!");
    }

    path_pub_->publish(path);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
