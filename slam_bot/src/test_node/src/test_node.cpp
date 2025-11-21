
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Define the namespace for the component (Must match the exported component namespace)
namespace test_node
{

  // Create a class that inherits from rclcpp::Node
  class TestNode : public rclcpp::Node
  {
  public:
    // Constructor requires 'options' for node initialization
    // The node name should typically be the executable/component name
    TestNode(const rclcpp::NodeOptions &options)
        : rclcpp::Node("test_node", options) 
    {
      RCLCPP_INFO(this->get_logger(), "We Did It Node has started!");

      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TestNode::timer_callback, this)
    );
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    int tick_count_ = 0;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // The function executed every 1 second
    void timer_callback()
{
  geometry_msgs::msg::Twist cmd;

  // Each movement lasts 20 ticks (2 seconds at 100ms)
  // Each gap STOP lasts 5 ticks (0.5 seconds)
  int phase = tick_count_ / 25;   // 20 active + 5 stop = 25 total
  int step  = tick_count_ % 25;

  bool stopping = (step >= 20);   // last 5 ticks = STOP

  if (stopping) {
    // STOP phase
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.angular.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "Stop...");
  }
  else {
    // ACTIVE movement phases
    switch (phase) {
      case 0:
        cmd.linear.x = 0.6;
        RCLCPP_INFO(this->get_logger(), "Forward");
        break;

      case 1:
        cmd.linear.x = -0.6;
        RCLCPP_INFO(this->get_logger(), "Backward");
        break;

      case 2:
        cmd.linear.y = 0.6;
        RCLCPP_INFO(this->get_logger(), "Strafe Left");
        break;

      case 3:
        cmd.linear.y = -0.6;
        RCLCPP_INFO(this->get_logger(), "Strafe Right");
        break;

      case 4:
        cmd.angular.z = 0.6;
        RCLCPP_INFO(this->get_logger(), "Rotate Left");
        break;

      case 5:
        cmd.angular.z = -0.6;
        RCLCPP_INFO(this->get_logger(), "Rotate Right");
        break;

      case 6:
        cmd.linear.x = 0.6;
        cmd.linear.y = 0.6;
        RCLCPP_INFO(this->get_logger(), "Diagonal Forward-Left");
        break;

      case 7:
        cmd.linear.x = -0.6;
        cmd.linear.y = -0.6;
        RCLCPP_INFO(this->get_logger(), "Diagonal Back-Right");
        break;

      default:
        // End of sequence → full stop
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Sequence Complete — STOP");
        break;
    }
  }

  publisher_->publish(cmd);
  tick_count_++;
}

  }; // class TestNode
  
} // namespace test_node

// Export the component using the class and namespace. This is required for composability.
// The macro looks for 'test_node::TestNode'
RCLCPP_COMPONENTS_REGISTER_NODE(test_node::TestNode)
