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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "cpp_test/text_publisher_component.hpp"

class TextPublisherShim : public cpp_test::TextPublisher
{
public:
  TextPublisherShim()
  : cpp_test::TextPublisher(rclcpp::NodeOptions())
  {
  }

  void setDynamicCallback()
  {
    auto node = shared_from_this();

    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(&TextPublisherShim::dynamicParamsShim, this, std::placeholders::_1));
  }

  rcl_interfaces::msg::SetParametersResult
  dynamicParamsShim(std::vector<rclcpp::Parameter> parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    dynamicParametersCallback(parameters);
    return result;
  }
};

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(WPTest, test_dynamic_parameters)
{
  auto text_publisher = std::make_shared<TextPublisherShim>();
  text_publisher->setDynamicCallback();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    text_publisher->get_node_base_interface(), text_publisher->get_node_topics_interface(),
    text_publisher->get_node_graph_interface(), text_publisher->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("message", "test message!")});

  rclcpp::spin_until_future_complete(text_publisher->get_node_base_interface(), results);

  EXPECT_EQ(text_publisher->get_parameter("message").as_string(), "test message!");

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("message", "hello test")});

  rclcpp::spin_until_future_complete(text_publisher->get_node_base_interface(), results);

  EXPECT_EQ(text_publisher->get_parameter("message").as_string(), "hello test");
}
