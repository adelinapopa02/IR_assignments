#include "rclcpp/rclcpp.hpp"
#include "group10_ex4/srv/refill_burrow.hpp" 
#include <cstdlib>
#include <ctime>

using RefillBurrow = group10_ex4::srv::RefillBurrow;
using namespace std::chrono_literals;

class BurrowClient : public rclcpp::Node
{
public:
    BurrowClient() : Node("burrow_client_node")
    {
        client_ = this->create_client<RefillBurrow>("refill_burrow_service");

        // Use a timer to periodically send requests and explore scenarios
        timer_ = this->create_wall_timer(
            5s, std::bind(&BurrowClient::send_refill_request, this));

        // Seed random number generator for scenario testing
        std::srand(std::time(nullptr));
    }

private:
    rclcpp::Client<RefillBurrow>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int scenario_counter_ = 0;

    void send_refill_request()
    {
        // 1. Wait for service to be available
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Refill service not available. Waiting...");
            return;
        }

        // 2. Setup the request for various scenarios
        auto request = std::make_shared<RefillBurrow::Request>();
        
        // Scenario Cycling for testing (n < s must be met for n<s case)
        if (scenario_counter_ % 3 == 0) {
            // SCENARIO 1: Standard Need (n < s, significant need)
            request->burrow_size = 10;
            request->n_apples = std::rand() % 4; // n=0, 1, 2, 3. Needed: 6-10.
            RCLCPP_INFO(this->get_logger(), "--- SCENARIO 1: Significant Need ---");
        } else if (scenario_counter_ % 3 == 1) {
            // SCENARIO 2: Full or Almost Full (n ~= s)
            request->burrow_size = 10;
            request->n_apples = 9; // Needed: 1.
            RCLCPP_INFO(this->get_logger(), "--- SCENARIO 2: Minimal Need (n ~= s) ---");
        } else {
            // SCENARIO 3: Large Need (n << s)
            request->burrow_size = 20;
            request->n_apples = 1; // Needed: 19.
            RCLCPP_INFO(this->get_logger(), "--- SCENARIO 3: Very Large Need (n < s) ---");
        }
        
        scenario_counter_++;

        // 3. Send the request asynchronously (safer than synchronous blocking)
        auto result_future = client_->async_send_request(request);

        // 4. Wait for the response (makes the node synchronous for this call)
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Received response: Can Refill = %s",
                        response->can_refill ? "TRUE" : "FALSE");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed! TurtleBot crashed?");
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BurrowClient>());
    rclcpp::shutdown();
    return 0;
}