#ifndef TESTROS2_HPP
#define TESTROS2_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace TCAS {
class PublishingSubscriber : public rclcpp::Node {
 public:
  PublishingSubscriber();

 private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
}  // namespace TCAS

#endif  // TESTROS2_HPP