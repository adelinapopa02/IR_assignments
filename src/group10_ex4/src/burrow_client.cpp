#include "rclcpp/rclcpp.hpp"
#include "group10_ex4/srv/check_resources.hpp" // Your custom service header
#include <cstdlib>
#include <ctime>
#include <chrono>

using CheckResources = group10_ex4::srv::CheckResources;
using namespace std::chrono_literals;

class BurrowClient : public rclcpp::Node {
public:
    BurrowClient() : Node("burrow_client_node") {
        // 1. Service Client setup
        client_ = this->create_client<CheckResources>("check_burrow_resources");
        RCLCPP_INFO(this->get_logger(), "Service Client created.");

        // Wait for the service server to be available
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service 'check_burrow_resources'...");
        }

        // 2. Timer to periodically call the service for different scenarios
        timer_ = this->create_wall_timer(
            5s, std::bind(&BurrowClient::call_service, this));
        
        std::srand(std::time(nullptr)); // Seed random number generator
    }

private:
    rclcpp::Client<CheckResources>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void call_service() {
        auto request = std::make_shared<CheckResources::Request>();
        
        // Randomly generate n and s such that n < s [cite: 38]
        int s = (std::rand() % 10) + 5; // Burrow size (s) between 5 and 14
        int n = std::rand() % s;        // Current apples (n) between 0 and s-1
        
        request->burrow_size_s = s;
        request->current_apples_n = n;
        
        RCLCPP_INFO(this->get_logger(), "\n--- Client Request Sent ---");
        RCLCPP_INFO(this->get_logger(), "Requesting Turtlebot: n=%d, s=%d. Needs %d apples.", 
            n, s, s - n);

        // Send the request
        auto result_future = client_->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            
            auto response = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service Response Received: Can refill burrow: %s", 
                response->can_refill_burrow ? "TRUE" : "FALSE");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service 'check_burrow_resources'");
        }
        RCLCPP_INFO(this->get_logger(), "---------------------------\n");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BurrowClient>());
    rclcpp::shutdown();
    return 0;
}