#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_node");
  auto pub = node->create_publisher<std_msgs::msg::Int32>("num", 10);

  rclcpp::WallRate loop(1);
  std_msgs::msg::Int32 msg;
  msg.data = 0;

  while(rclcpp::ok()){
    if(msg.data < 10){
      msg.data++;
    }else{
      msg.data = 0;
    }

    pub->publish(msg);
    loop.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
