#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PlaceholderNode : public rclcpp::Node
{
public:
  PlaceholderNode()
      : Node("placeholder_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&PlaceholderNode::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlaceholderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}