#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "group10_ex4/srv/check_resources.hpp" // Your custom service header
#include <cmath>

using CheckResources = group10_ex4::srv::CheckResources;
using PoseArray = geometry_msgs::msg::PoseArray;
using LaserScan = sensor_msgs::msg::LaserScan;

class TurtlebotServer : public rclcpp::Node {
public:
    TurtlebotServer() : Node("turtlebot_server_node"), current_apple_count_(0) {
        // 1. Service Server setup
        service_ = this->create_service<CheckResources>(
            "check_burrow_resources",
            std::bind(&TurtlebotServer::handle_service_request, this,
                      std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Service Server ready: 'check_burrow_resources'");

        // 2. Lidar Subscriber setup
        scan_subscriber_ = this->create_subscription<LaserScan>(
            "scan", 10,
            std::bind(&TurtlebotServer::scan_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribing to Lidar topic: '/scan'");

        // 3. Apple Poses Publisher setup
        apples_publisher_ = this->create_publisher<PoseArray>("/apples", 10);
        RCLCPP_INFO(this->get_logger(), "Publishing to topic: '/apples'");
    }

private:
    rclcpp::Service<CheckResources>::SharedPtr service_;
    rclcpp::Subscription<LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<PoseArray>::SharedPtr apples_publisher_;
    int current_apple_count_;

    // --- Lidar Processing and Apple Detection ---
    void scan_callback(const LaserScan::SharedPtr msg) {
        // Simple Lidar processing to count apples (spheres)
        // **This is a simplified approach. A more robust solution requires clustering.**
        
        // Define parameters (Tune these based on the Gazebo world)
        const float MAX_RANGE_FOR_APPLE = 0.6; // Max distance to consider for a close object (apple)
        const float MIN_RANGE = 0.1;           // Ignore points too close
        const int MIN_POINTS_FOR_APPLE = 3;    // Minimum consecutive points for a cluster
        const float CLUSTER_BREAK_DIFF = 0.1;  // Max distance difference to break a cluster

        std::vector<PoseArray> apple_clusters;
        std::vector<int> current_cluster_indices;
        
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];

            if (range > MIN_RANGE && range < MAX_RANGE_FOR_APPLE && !std::isinf(range)) {
                // Point is a potential part of an apple
                if (!current_cluster_indices.empty()) {
                    // Check if this point continues the previous cluster
                    float prev_range = msg->ranges[current_cluster_indices.back()];
                    if (std::abs(range - prev_range) < CLUSTER_BREAK_DIFF) {
                        current_cluster_indices.push_back(i);
                    } else {
                        // Cluster ended. Process the previous one.
                        process_cluster(msg, current_cluster_indices, apple_clusters);
                        current_cluster_indices.clear();
                        current_cluster_indices.push_back(i); // Start new cluster
                    }
                } else {
                    current_cluster_indices.push_back(i); // Start of a new cluster
                }
            } else {
                // Point is too far or too close. Cluster must end.
                process_cluster(msg, current_cluster_indices, apple_clusters);
                current_cluster_indices.clear();
            }
        }
        // Process the last cluster if it wasn't processed
        process_cluster(msg, current_cluster_indices, apple_clusters);

        // Update the count and publish poses
        current_apple_count_ = apple_clusters.size();
        
        PoseArray poses_msg;
        poses_msg.header.frame_id = "base_link"; // Apples pose relative to base_link [cite: 41]
        poses_msg.header.stamp = this->now();

        for (const auto& apple_cluster : apple_clusters) {
            // Since we stored the average pose in the cluster's first pose, we extract it here
            poses_msg.poses.push_back(apple_cluster.poses[0]); 
        }

        apples_publisher_->publish(poses_msg);
    }
    
    // Helper function to process a finished cluster
    void process_cluster(const LaserScan::SharedPtr& msg, 
                         const std::vector<int>& cluster_indices, 
                         std::vector<PoseArray>& apple_clusters) {
        const int MIN_POINTS_FOR_APPLE = 3; // Use the same const as above
        if (cluster_indices.size() >= MIN_POINTS_FOR_APPLE) {
            // Found an apple! Convert points to Cartesian and find the centroid
            double x_sum = 0.0;
            double y_sum = 0.0;
            
            for (int index : cluster_indices) {
                // Calculate angle for the index: angle_min + index * angle_increment
                double angle = msg->angle_min + index * msg->angle_increment;
                double range = msg->ranges[index];
                
                // Convert Polar (range, angle) to Cartesian (x, y)
                double x = range * std::cos(angle);
                double y = range * std::sin(angle);
                
                x_sum += x;
                y_sum += y;
            }
            
            // Calculate Centroid (Estimated Apple Position)
            double x_center = x_sum / cluster_indices.size();
            double y_center = y_sum / cluster_indices.size();
            
            PoseArray cluster_pose;
            cluster_pose.poses.resize(1);
            cluster_pose.poses[0].position.x = x_center;
            cluster_pose.poses[0].position.y = y_center;
            // Orientation is typically not calculated for 2D Lidar clusters, so leave Z and W at 0/1
            cluster_pose.poses[0].orientation.w = 1.0; 
            
            apple_clusters.push_back(cluster_pose);
        }
    }


    // --- Service Handler ---
    void handle_service_request(
        const std::shared_ptr<CheckResources::Request> request,
        const std::shared_ptr<CheckResources::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "\n--- Service Request Received ---");
        int n = request->current_apples_n;
        int s = request->burrow_size_s;
        
        RCLCPP_INFO(this->get_logger(), "Burrow state: current apples n=%d, size s=%d.", n, s);

        int needed_apples = s - n;
        RCLCPP_INFO(this->get_logger(), "Needed apples (s - n): %d", needed_apples);
        RCLCPP_INFO(this->get_logger(), "Apples found by Turtlebot (Lidar): %d", current_apple_count_);

        // Determine if enough apples were found [cite: 40]
        if (current_apple_count_ >= needed_apples) {
            response->can_refill_burrow = true;
            RCLCPP_INFO(this->get_logger(), "Result: Enough apples found to refill the burrow! (TRUE)");
        } else {
            response->can_refill_burrow = false;
            RCLCPP_INFO(this->get_logger(), "Result: Not enough apples found. (FALSE)");
        }
        
        RCLCPP_INFO(this->get_logger(), "--------------------------------");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotServer>());
    rclcpp::shutdown();
    return 0;
}