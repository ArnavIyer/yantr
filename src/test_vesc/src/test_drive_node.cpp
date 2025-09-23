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
        test_phase_ = TestPhase::DRIVING;
        
        RCLCPP_INFO(this->get_logger(), "Test Drive Node initialized");
        RCLCPP_INFO(this->get_logger(), "Starting 3-second drive test at 1.5 m/s");
        
        // Publish initial status
        publishStatus("TEST STARTED: Driving at 1.5 m/s for 3 seconds");
        
        // Create timer to control the test sequence (50Hz for smooth control)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&TestDriveNode::timerCallback, this));
    }

private:
    enum class TestPhase
    {
        DRIVING,      // Drive at 1.5 m/s for 3 seconds
        STOPPING,     // Send stop command for 1 second
        FINISHED      // Test complete
    };

    void timerCallback()
    {
        auto current_time = this->get_clock()->now();
        auto elapsed_time = (current_time - test_start_time_).seconds();
        
        geometry_msgs::msg::Twist cmd_vel_msg;
        
        switch (test_phase_)
        {
            case TestPhase::DRIVING:
                if (elapsed_time < 3.0)
                {
                    // Drive forward at 1.5 m/s
                    cmd_vel_msg.linear.x = 1.5;
                    cmd_vel_msg.linear.y = 0.0;
                    cmd_vel_msg.linear.z = 0.0;
                    cmd_vel_msg.angular.x = 0.0;
                    cmd_vel_msg.angular.y = 0.0;
                    cmd_vel_msg.angular.z = 0.0;
                    
                    cmd_vel_publisher_->publish(cmd_vel_msg);
                    
                    // Log progress every 0.5 seconds
                    if (static_cast<int>(elapsed_time * 2) != last_log_count_)
                    {
                        last_log_count_ = static_cast<int>(elapsed_time * 2);
                        RCLCPP_INFO(this->get_logger(), 
                                   "Driving at 1.5 m/s... %.1f/3.0 seconds", elapsed_time);
                        publishStatus("DRIVING: " + std::to_string(elapsed_time) + "/3.0 seconds at 1.5 m/s");
                    }
                }
                else
                {
                    // Switch to stopping phase
                    test_phase_ = TestPhase::STOPPING;
                    stop_start_time_ = current_time;
                    RCLCPP_INFO(this->get_logger(), "3 seconds complete! Sending stop command...");
                    publishStatus("STOPPING: Sending explicit stop command (0 m/s)");
                }
                break;
                
            case TestPhase::STOPPING:
                {
                    auto stop_elapsed = (current_time - stop_start_time_).seconds();
                    if (stop_elapsed < 1.0)
                    {
                        // Send explicit stop command
                        cmd_vel_msg.linear.x = 0.0;
                        cmd_vel_msg.linear.y = 0.0;
                        cmd_vel_msg.linear.z = 0.0;
                        cmd_vel_msg.angular.x = 0.0;
                        cmd_vel_msg.angular.y = 0.0;
                        cmd_vel_msg.angular.z = 0.0;
                        
                        cmd_vel_publisher_->publish(cmd_vel_msg);
                    }
                    else
                    {
                        // Switch to finished phase
                        test_phase_ = TestPhase::FINISHED;
                        RCLCPP_INFO(this->get_logger(), "Test complete! Motor should be stopped.");
                        RCLCPP_INFO(this->get_logger(), "Note: VESC driver has 0.5s timeout safety - motor will stop automatically if no commands sent");
                        publishStatus("TEST COMPLETE: No more commands will be sent. Motor stopped via timeout safety.");
                    }
                }
                break;
                
            case TestPhase::FINISHED:
                // Do nothing - test is complete
                // The timer will keep running but no commands will be sent
                // This allows the VESC timeout safety to take effect
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
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting VESC Test Drive Node");
    RCLCPP_INFO(rclcpp::get_logger("main"), "This will drive the robot at 1.5 m/s for 3 seconds, then stop");
    RCLCPP_INFO(rclcpp::get_logger("main"), "Make sure the vesc_driver_node is running and the robot has clearance!");
    
    auto node = std::make_shared<TestDriveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}