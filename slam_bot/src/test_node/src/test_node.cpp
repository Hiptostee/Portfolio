
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

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

      timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&TestNode::timer_callback, this));
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;

    // The function executed every 1 second
    void timer_callback()
    {
      RCLCPP_INFO(this->get_logger(), "WE DID IT! 🎉");
    }
  };

} // namespace test_node

// Export the component using the class and namespace. This is required for composability.
// The macro looks for 'test_node::TestNode'
RCLCPP_COMPONENTS_REGISTER_NODE(test_node::TestNode)
