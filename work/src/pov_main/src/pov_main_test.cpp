#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

class PovMainTestNode : public rclcpp::Node {
public:
  PovMainTestNode() : Node("pov_main_test_node"), publish_value_("Bad view") {
    result_pub_ = this->create_publisher<std_msgs::msg::String>("/pov_demo/hailo8/result", 10);

    command_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/pov_test/pov_main_test/command", 10,
        std::bind(&PovMainTestNode::command_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PovMainTestNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "PovMainTestNode started");
  }

private:
  void command_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    publish_value_ = msg->data ? "Good view" : "Bad view";
    RCLCPP_INFO(this->get_logger(), "Received command: %s, will publish: %s",
                msg->data ? "true" : "false", publish_value_.c_str());
  }

  void timer_callback() {
    std_msgs::msg::String msg;
    msg.data = publish_value_;
    result_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published /pov_demo/hailo8/result: %s", msg.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string publish_value_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PovMainTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}