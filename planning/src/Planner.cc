#include "Planner.h"

namespace TCAS {

Planner::Planner() : Node("Planner") {
  init();
  createAllSubscribers();
  createAllPublishers();
}

void Planner::init() {
  number_of_robots_ = countRobotTopics();
  robots_.resize(number_of_robots_);
  subscribers_.resize(number_of_robots_);
  robot_publishers_.resize(number_of_robots_);
  positions_.resize(number_of_robots_);
  orientations_.resize(number_of_robots_);
  distance_to_target_.resize(number_of_robots_);
  final_goal_.resize(number_of_robots_);

  for (int i = 0; i < number_of_robots_; ++i) {
    robots_[i] = "robot" + std::to_string(i);
    positions_[i] = geometry_msgs::msg::Point();
    orientations_[i] = 0.0;
    distance_to_target_[i] = 0.0;
    final_goal_[i] = geometry_msgs::msg::Point();
  }

  position_received_ = 0;
  first_time_planning_ = true;
  cbs_time_schedule_ = 1;
  number_of_successfully_executed_plans_ = 0;
  obstacles_ = std::make_pair(500, 500);  // Dummy obstacle
}
void Planner::createAllSubscribers() {}

void Planner::createAllPublishers() {}
unsigned short Planner::countRobotTopics() {
  auto topic_list = this->get_topic_names_and_types();
  unsigned short robot_count = 0;

  for (const auto& item : topic_list) {
    if (item.first.find("/robot") == 0 &&
        item.first.find("/cmd_vel") != std::string::npos) {
      robot_count++;
    }
  }

  return robot_count;
}

}  // namespace TCAS
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TCAS::Planner>());
  rclcpp::shutdown();
  return 0;
}