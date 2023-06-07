#include "Planner.h"

namespace TCAS {

Planner::Planner() : Node("Planner") {
  RCLCPP_INFO(get_logger(), "Planner Constructor has been called");
}

void Planner::init() {}
void Planner::createAllSubscribers() {}
void Planner::positionCallback(const nav_msgs::msg::Odometry::SharedPtr msg,
                               const std::string& topic) {}
void Planner::readCustomGoals() {}

void Planner::goalCallback(const geometry_msgs::msg::Point::SharedPtr msg,
                           const std::string& topic) {}

bool Planner::allRobotsArrivedCBSFinalWaypoint() {}

bool Planner::allRobotsArrivedInWaypoints() {}

const geometry_msgs::msg::Point& Planner::getNextTargetWaypoints(
    const int robot_index) {}

int Planner::getRobotIndex(const std::string& robot_name) const {}
void Planner::createAllPublishers() {}
int Planner::countRobotTopics() {}

bool Planner::updateObstacleLocations() {}

void Planner::updateGoal(int robot_id,
                         const geometry_msgs::msg::Point& new_goal) {}

void Planner::driveRobotstoCbsWaypoints() {}
void Planner::callCbsPlanner() {}

void Planner::haltRobots() {}
void Planner::commandRobot(int robot_index,
                           const geometry_msgs::msg::Point& target_waypoint,
                           double current_orientation) {}

bool Planner::allPositionsReceived() {}
void Planner::resolveGoalConflicts() {}

geometry_msgs::msg::Point Planner::generateNewGoal(
    const geometry_msgs::msg::Point& old_goal) {}
void Planner::resolveObstacleConflicts() {}

void Planner::verifyInitialRobotPositions() {}

float Planner::getDistance(const float dx, const float dy) {}
std::string Planner::getFullFilename(const std::string& paramFilename) {}

void Planner::writeDataToYaml(std::string& filename) {}
void Planner::getDataFromYaml(std::string& filename) {}

void Planner::generateNewTargets(unsigned int robot_index,
                                 geometry_msgs::msg::Point new_goal) {}

}  // namespace TCAS
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto plannerNode = std::make_shared<TCAS::Planner>();
  rclcpp::spin(plannerNode);
  rclcpp::shutdown();
  return 0;
}