#include "cpp_test/text_publisher_component.hpp"

namespace cpp_test
{

TextPublisher::TextPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("text_publisher_node", options)
{
  this->declare_parameter<std::string>("message", "hello world.");
  this->get_parameter("message", message_);
  
  pub_text_ = this->create_publisher<std_msgs::msg::String>("text", 1);
  timer_ = this->create_wall_timer(100ms, std::bind(&TextPublisher::timerCallback, this));
}

TextPublisher::~TextPublisher()
{
}

void TextPublisher::timerCallback()
{
  auto text = std::make_unique<std_msgs::msg::String>();
  
  text->data = message_;
  pub_text_->publish(std::move(text));
}

} // namepsace cpp_test

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cpp_test::TextPublisher)
