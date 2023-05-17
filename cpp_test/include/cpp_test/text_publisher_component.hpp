#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace cpp_test
{

class TextPublisher : public rclcpp::Node
{
public:
  explicit TextPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TextPublisher();
  
private:
  void timerCallback();
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_text_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::string message_;
};

} // namespace cpp_test
