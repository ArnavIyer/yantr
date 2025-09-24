#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

class TestDriveNode : public rclcpp::Node
{
public:
    TestDriveNode() : Node("test_drive_node")
    {
        // Create publisher for cmd_vel topic
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Create status publisher for logging
        status_publisher_ = this->create_publisher<std_msgs::msg::String>("test_drive_status", 10);
        
        // Initialize state
        test_start_time_ = this->get_clock()->now();
        test_phase_ = TestPhase::FORWARD;
        
        RCLCPP_INFO(this->get_logger(), "Skid-Steer Test Drive Node initialized");
        RCLCPP_INFO(this->get_logger(), "Test Sequence:");
        RCLCPP_INFO(this->get_logger(), "  1. Forward at 1.0 m/s for 3s");
        RCLCPP_INFO(this->get_logger(), "  2. Backward at -0.5 m/s for 2s");
        RCLCPP_INFO(this->get_logger(), "  3. Turn left in place for 2s");
        RCLCPP_INFO(this->get_logger(), "  4. Turn right in place for 2s");
        RCLCPP_INFO(this->get_logger(), "  5. Combined forward + turn for 3s");
        RCLCPP_INFO(this->get_logger(), "  6. Stop for 1s");
        
        // Publish initial status
        publishStatus("SKID-STEER TEST STARTED: Forward motion at 1.0 m/s for 3 seconds");
        
        // Create timer to control the test sequence (50Hz for smooth control)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&TestDriveNode::timerCallback, this));
    }

private:
    enum class TestPhase
    {
        FORWARD,       // Drive forward at 1.0 m/s for 3 seconds
        BACKWARD,      // Drive backward at -0.5 m/s for 2 seconds
        TURN_LEFT,     // Turn left in place for 2 seconds
        TURN_RIGHT,    // Turn right in place for 2 seconds
        COMBINED,      // Combined forward + turn for 3 seconds
        STOPPING,      // Send stop command for 1 second
        FINISHED       // Test complete
    };

    void timerCallback()
    {
        auto current_time = this->get_clock()->now();
        auto elapsed_time = (current_time - test_start_time_).seconds();
        
        geometry_msgs::msg::Twist cmd_vel_msg;
        
        switch (test_phase_)
        {
            case TestPhase::FORWARD:
                if (elapsed_time < 3.0)
                {
                    // Drive forward at 1.0 m/s
                    cmd_vel_msg.linear.x = 1.0;
                    cmd_vel_msg.angular.z = 0.0;
                    cmd_vel_publisher_->publish(cmd_vel_msg);
                    
                    if (static_cast<int>(elapsed_time * 2) != last_log_count_)
                    {
                        last_log_count_ = static_cast<int>(elapsed_time * 2);
                        RCLCPP_INFO(this->get_logger(), 
                                   "FORWARD: %.1f/3.0 seconds at 1.0 m/s", elapsed_time);
                        publishStatus("FORWARD: " + std::to_string(elapsed_time) + "/3.0 seconds");
                    }
                }
                else
                {
                    test_phase_ = TestPhase::BACKWARD;
                    test_start_time_ = current_time;
                    last_log_count_ = -1;
                    RCLCPP_INFO(this->get_logger(), "Forward complete! Starting backward motion...");
                    publishStatus("BACKWARD: Starting -0.5 m/s for 2 seconds");
                }
                break;
                
            case TestPhase::BACKWARD:
                if (elapsed_time < 2.0)
                {
                    // Drive backward at -0.5 m/s
                    cmd_vel_msg.linear.x = -0.5;
                    cmd_vel_msg.angular.z = 0.0;
                    cmd_vel_publisher_->publish(cmd_vel_msg);
                    
                    if (static_cast<int>(elapsed_time * 2) != last_log_count_)
                    {
                        last_log_count_ = static_cast<int>(elapsed_time * 2);
                        RCLCPP_INFO(this->get_logger(), 
                                   "BACKWARD: %.1f/2.0 seconds at -0.5 m/s", elapsed_time);
                        publishStatus("BACKWARD: " + std::to_string(elapsed_time) + "/2.0 seconds");
                    }
                }
                else
                {
                    test_phase_ = TestPhase::TURN_LEFT;
                    test_start_time_ = current_time;
                    last_log_count_ = -1;
                    RCLCPP_INFO(this->get_logger(), "Backward complete! Starting left turn...");
                    publishStatus("TURN_LEFT: Starting in-place left turn for 2 seconds");
                }
                break;
                
            case TestPhase::TURN_LEFT:
                if (elapsed_time < 2.0)
                {
                    // Turn left in place (positive angular velocity)
                    cmd_vel_msg.linear.x = 0.0;
                    cmd_vel_msg.angular.z = 1.0;  // rad/s
                    cmd_vel_publisher_->publish(cmd_vel_msg);
                    
                    if (static_cast<int>(elapsed_time * 2) != last_log_count_)
                    {
                        last_log_count_ = static_cast<int>(elapsed_time * 2);
                        RCLCPP_INFO(this->get_logger(), 
                                   "TURN_LEFT: %.1f/2.0 seconds at 1.0 rad/s", elapsed_time);
                        publishStatus("TURN_LEFT: " + std::to_string(elapsed_time) + "/2.0 seconds");
                    }
                }
                else
                {
                    test_phase_ = TestPhase::TURN_RIGHT;
                    test_start_time_ = current_time;
                    last_log_count_ = -1;
                    RCLCPP_INFO(this->get_logger(), "Left turn complete! Starting right turn...");
                    publishStatus("TURN_RIGHT: Starting in-place right turn for 2 seconds");
                }
                break;
                
            case TestPhase::TURN_RIGHT:
                if (elapsed_time < 2.0)
                {
                    // Turn right in place (negative angular velocity)
                    cmd_vel_msg.linear.x = 0.0;
                    cmd_vel_msg.angular.z = -1.0;  // rad/s
                    cmd_vel_publisher_->publish(cmd_vel_msg);
                    
                    if (static_cast<int>(elapsed_time * 2) != last_log_count_)
                    {
                        last_log_count_ = static_cast<int>(elapsed_time * 2);
                        RCLCPP_INFO(this->get_logger(), 
                                   "TURN_RIGHT: %.1f/2.0 seconds at -1.0 rad/s", elapsed_time);
                        publishStatus("TURN_RIGHT: " + std::to_string(elapsed_time) + "/2.0 seconds");
                    }
                }
                else
                {
                    test_phase_ = TestPhase::COMBINED;
                    test_start_time_ = current_time;
                    last_log_count_ = -1;
                    RCLCPP_INFO(this->get_logger(), "Right turn complete! Starting combined motion...");
                    publishStatus("COMBINED: Starting forward + turn motion for 3 seconds");
                }
                break;
                
            case TestPhase::COMBINED:
                if (elapsed_time < 3.0)
                {
                    // Combined forward motion with turning (driving in a curve)
                    cmd_vel_msg.linear.x = 0.8;   // m/s forward
                    cmd_vel_msg.angular.z = 0.5;  // rad/s turning
                    cmd_vel_publisher_->publish(cmd_vel_msg);
                    
                    if (static_cast<int>(elapsed_time * 2) != last_log_count_)
                    {
                        last_log_count_ = static_cast<int>(elapsed_time * 2);
                        RCLCPP_INFO(this->get_logger(), 
                                   "COMBINED: %.1f/3.0 seconds - forward + turn", elapsed_time);
                        publishStatus("COMBINED: " + std::to_string(elapsed_time) + "/3.0 seconds");
                    }
                }
                else
                {
                    test_phase_ = TestPhase::STOPPING;
                    stop_start_time_ = current_time;
                    RCLCPP_INFO(this->get_logger(), "Combined motion complete! Sending stop command...");
                    publishStatus("STOPPING: Sending explicit stop command");
                }
                break;
                
            case TestPhase::STOPPING:
                {
                    auto stop_elapsed = (current_time - stop_start_time_).seconds();
                    if (stop_elapsed < 1.0)
                    {
                        // Send explicit stop command
                        cmd_vel_msg.linear.x = 0.0;
                        cmd_vel_msg.angular.z = 0.0;
                        cmd_vel_publisher_->publish(cmd_vel_msg);
                    }
                    else
                    {
                        test_phase_ = TestPhase::FINISHED;
                        RCLCPP_INFO(this->get_logger(), "SKID-STEER TEST COMPLETE!");
                        RCLCPP_INFO(this->get_logger(), "All motion patterns tested:");
                        RCLCPP_INFO(this->get_logger(), "  ✓ Forward motion");
                        RCLCPP_INFO(this->get_logger(), "  ✓ Backward motion");
                        RCLCPP_INFO(this->get_logger(), "  ✓ Left turning");
                        RCLCPP_INFO(this->get_logger(), "  ✓ Right turning");
                        RCLCPP_INFO(this->get_logger(), "  ✓ Combined motion");
                        RCLCPP_INFO(this->get_logger(), "Motors stopped via timeout safety.");
                        publishStatus("TEST COMPLETE: All skid-steer motions tested successfully!");
                    }
                }
                break;
                
            case TestPhase::FINISHED:
                // Test complete - no more commands
                break;
        }
    }
    
    void publishStatus(const std::string& message)
    {
        auto status_msg = std_msgs::msg::String();
        status_msg.data = message;
        status_publisher_->publish(status_msg);
    }

    // ROS2 interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Test state
    TestPhase test_phase_;
    rclcpp::Time test_start_time_;
    rclcpp::Time stop_start_time_;
    int last_log_count_ = -1;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting VESC Skid-Steer Test Drive Node");
    RCLCPP_INFO(rclcpp::get_logger("main"), "This will test all skid-steer motions:");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - Forward/backward motion");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - Left/right turning in place");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - Combined forward + turn motion");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Make sure VESC driver launch file is running and robot has clearance!");
    
    auto node = std::make_shared<TestDriveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}