// Copyright 2023 Yuma Matsumura All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cpp_test/text_publisher_component.hpp"

namespace cpp_test
{

TextPublisher::TextPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("text_publisher_node", options)
{
  using namespace std::chrono_literals;
  using namespace std::placeholders;

  this->declare_parameter<std::string>("message", "hello world.");
  this->get_parameter("message", message_);

  pub_text_ = this->create_publisher<std_msgs::msg::String>("text", 1);
  dyn_params_handler_ = this->add_on_set_parameters_callback(
    std::bind(&TextPublisher::dynamicParametersCallback, this, _1));
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

rcl_interfaces::msg::SetParametersResult
TextPublisher::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & name = parameter.get_name();

    if (name == "message") {
      message_ = parameter.as_string();
    }
  }

  result.successful = true;
  return result;
}

}  // namespace cpp_test

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cpp_test::TextPublisher)
