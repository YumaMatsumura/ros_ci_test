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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

namespace cpp_test
{

class TextPublisher : public rclcpp::Node
{
public:
  explicit TextPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TextPublisher();

protected:
  void timerCallback();
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_text_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string message_;
};

}  // namespace cpp_test
