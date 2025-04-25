#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <pov_demo_msg/msg/pov_result.hpp>

class PovMainTestNode : public rclcpp::Node {
public:
  PovMainTestNode() : Node("pov_main_test"), class_name_("Good view") {
    result_pub_ = this->create_publisher<pov_demo_msg::msg::PovResult>(
        "/pov_demo/hailo8/result", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PovMainTestNode::timer_callback, this));

    command_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/pov_test/pov_main_test/command", 10,
        std::bind(&PovMainTestNode::command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PovMainTestNode started");
  }

private:
  void timer_callback() {
    auto msg = pov_demo_msg::msg::PovResult();
    msg.class_name = class_name_;
    msg.prob = 0.9;

    result_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published /pov_demo/hailo8/result: class_name=%s, prob=%.2f",
                msg.class_name.c_str(), msg.prob);
  }

  void command_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    class_name_ = msg->data ? "Good view" : "Bad view";
    RCLCPP_INFO(this->get_logger(), "Received command: %s, class_name set to %s",
                msg->data ? "true" : "false", class_name_.c_str());
  }

  rclcpp::Publisher<pov_demo_msg::msg::PovResult>::SharedPtr result_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr command_sub_;
  std::string class_name_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PovMainTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}