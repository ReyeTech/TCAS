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
}

void Planner::createAllSubscribers() {
  // Create all subscribers
  // Add your subscriber creation code here
}

void Planner::createAllPublishers() {
  // Create all publishers
  // Add your publisher creation code here
}

}  // namespace TCAS
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TCAS::Planner>());
  rclcpp::shutdown();
  return 0;
}