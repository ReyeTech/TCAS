#include "Planner.h"
#define NUMBER_OF_ROBOTS 3
namespace TCAS {

Planner::Planner() : Node("robot_controller") { init(); }

void Planner::init() {
  RCLCPP_INFO(get_logger(), "Planner init has been called");
  number_of_robots_ = 0;
  while (number_of_robots_ < NUMBER_OF_ROBOTS) {
    number_of_robots_ = countRobotTopics();
    RCLCPP_INFO(get_logger(), "Waiting for robot topics odom and cmd_vel");
  }
  RCLCPP_INFO(get_logger(), "Number of robots is %d", number_of_robots_);
  robots_.resize(number_of_robots_);
  for (int i = 0; i < number_of_robots_; ++i) {
    robots_[i] = "robot" + std::to_string(i);
    RCLCPP_INFO(get_logger(), "Name of robot is %s\n", robots_[i].c_str());
  }
}
int Planner::countRobotTopics() {
  auto topic_list = this->get_topic_names_and_types();
  int robot_count_vel = 0;
  int robot_count_odom = 0;
  for (const auto& item : topic_list) {
    if (item.first.find("/robot") == 0 &&
        item.first.find("/cmd_vel") != std::string::npos) {
      robot_count_vel++;
    }
    if (item.first.find("/robot") == 0 &&
        item.first.find("/odom") != std::string::npos) {
      robot_count_odom++;
    }
  }
  if (robot_count_vel == NUMBER_OF_ROBOTS &&
      robot_count_odom == NUMBER_OF_ROBOTS) {
    return robot_count_odom;
  }

  return 0;
}
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