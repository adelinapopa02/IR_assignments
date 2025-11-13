#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "group10_ex4/srv/refill_burrow.hpp" 

using RefillBurrow = group10_ex4::srv::RefillBurrow;

class TurtleBotServer : public rclcpp::Node
{
public:
    TurtleBotServer() : Node("turtlebot_server_node"), found_apples_count_(0)
    {
        // 1. Service Server
        service_ = this->create_service<RefillBurrow>(
            "refill_burrow_service",
            std::bind(&TurtleBotServer::refill_burrow_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "TurtleBot Service Server ready.");

        // 2. LiDAR Subscriber
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&TurtleBotServer::lidar_callback, this, std::placeholders::_1));

        // 3. PoseArray Publisher
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/apples", 10);
    }

private:
    // Member variables
    rclcpp::Service<RefillBurrow>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_publisher_;
    int found_apples_count_;

    // --- Service Server Callback ---
    void refill_burrow_callback(
        const std::shared_ptr<RefillBurrow::Request> request,
        std::shared_ptr<RefillBurrow::Response> response)
    {
        int needed = request->burrow_size - request->n_apples;
        RCLCPP_INFO(this->get_logger(), "Burrow needs %d apples (n=%d, s=%d).",
                    needed, request->n_apples, request->burrow_size);
        
        // This is the core logic: compare what's needed with what's found
        if (needed <= found_apples_count_) {
            response->can_refill = true;
            RCLCPP_INFO(this->get_logger(), "Response: TRUE. Found %d apples, enough to refill.", found_apples_count_);
        } else {
            response->can_refill = false;
            RCLCPP_INFO(this->get_logger(), "Response: FALSE. Found only %d apples, not enough.", found_apples_count_);
        }
    }

    // --- LiDAR Subscriber Callback ---
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Implement apple detection and pose publishing here (Step 4 logic)
        std::vector<geometry_msgs::msg::Pose> apple_poses = detect_apples(msg);
        found_apples_count_ = apple_poses.size();
        
        geometry_msgs::msg::PoseArray pose_array_msg;
        pose_array_msg.header.stamp = this->get_clock()->now();
        pose_array_msg.header.frame_id = "base_link"; // Critical: poses relative to base_link
        pose_array_msg.poses = apple_poses;

        pose_publisher_->publish(pose_array_msg);
    }

    // This is the refined implementation for TurtleBotServer::detect_apples
    std::vector<geometry_msgs::msg::Pose> TurtleBotServer::detect_apples(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<geometry_msgs::msg::Pose> apple_poses;
        // Parameters to tune based on the Gazebo environment (e.g., apple size)
        const float MAX_RANGE_TO_CONSIDER = 1.0; // Only look 1m around the robot
        const int MIN_CLUSTER_SIZE = 3;          // Minimum contiguous points to form an apple
        const int MAX_CLUSTER_SIZE = 10;         // Maximum contiguous points

        size_t i = 0;
        while (i < msg->ranges.size()) {
            // 1. Find the start of a potential cluster (range less than threshold)
            if (msg->ranges[i] < MAX_RANGE_TO_CONSIDER && msg->ranges[i] > msg->range_min) {
                size_t start_index = i;
                size_t cluster_size = 0;
                float min_range = msg->range_max;
                size_t min_range_index = start_index;

                // 2. Grow the cluster
                while (i < msg->ranges.size() && 
                    msg->ranges[i] < MAX_RANGE_TO_CONSIDER && 
                    msg->ranges[i] > msg->range_min) 
                {
                    if (msg->ranges[i] < min_range) {
                        min_range = msg->ranges[i];
                        min_range_index = i;
                    }
                    cluster_size++;
                    i++;
                }

                // 3. Check if the cluster size is plausible for an apple
                if (cluster_size >= MIN_CLUSTER_SIZE && cluster_size <= MAX_CLUSTER_SIZE) {
                    // We found a good cluster. Calculate the pose using the closest point.
                    geometry_msgs::msg::Pose apple_pose;
                    
                    // Calculate angle for the closest point
                    float angle = msg->angle_min + min_range_index * msg->angle_increment;
                    
                    // Convert polar (range, angle) to Cartesian (x, y)
                    apple_pose.position.x = min_range * cos(angle);
                    apple_pose.position.y = min_range * sin(angle);
                    apple_pose.position.z = 0.0; // The apple is on the floor
                    apple_pose.orientation.w = 1.0; // No rotation needed for a sphere marker
                    
                    apple_poses.push_back(apple_pose);
                }
            }
            i++;
        }

        RCLCPP_INFO(this->get_logger(), "LiDAR: Detected %zu apples.", apple_poses.size());
        return apple_poses;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleBotServer>());
    rclcpp::shutdown();
    return 0;
}