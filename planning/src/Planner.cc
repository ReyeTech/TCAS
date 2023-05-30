#include "Planner.h"

namespace TCAS {

Planner::Planner() : Node("Planner") {
  init();
  createAllSubscribers();
  createAllPublishers();
}

void Planner::init() {
  // Initialize the Planner object
  // Add your initialization code here
  number_of_robots_ = countRobotTopics();
}

void Planner::createAllSubscribers() {
  // Create all subscribers
  // Add your subscriber creation code here
}

void Planner::createAllPublishers() {
  // Create all publishers
  // Add your publisher creation code here
}
unsigned short Planner::countRobotTopics() {
  // Count number of robot topics to know the number of robots avoiding
  // hardcoding it here. Returns a short
  return 0;
}

}  // namespace TCAS
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TCAS::Planner>());
  rclcpp::shutdown();
  return 0;
}