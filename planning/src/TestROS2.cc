#include "TestROS2.h"

namespace TCAS {
PublishingSubscriber::PublishingSubscriber() : Node("publishing_subscriber") {
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "addison", 10,
      std::bind(&PublishingSubscriber::topic_callback, this,
                std::placeholders::_1));

  publisher_ = this->create_publisher<std_msgs::msg::String>("addison2", 10);
}

void PublishingSubscriber::topic_callback(
    const std_msgs::msg::String::SharedPtr msg) const {
  auto message = std_msgs::msg::String();
  message.data = "I heard " + msg->data;
  publisher_->publish(message);
}
}  // namespace TCAS

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TCAS::PublishingSubscriber>());
  rclcpp::shutdown();
  return 0;
}