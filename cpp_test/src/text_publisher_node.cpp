#include "rclcpp/rclcpp.hpp"
#include "cpp_test/text_publisher_component.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto node = std::make_shared<cpp_test::TextPublisher>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  
  return 0;
}
