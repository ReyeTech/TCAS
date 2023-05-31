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
void Planner::createAllSubscribers() {
  for (const auto& robot_name : robots_) {
    std::string topic = "/" + robot_name + "/odom";
    auto subscriber = create_subscription<nav_msgs::msg::Odometry>(
        topic, 10,
        [this, topic,
         robot_name](const nav_msgs::msg::Odometry::SharedPtr msg) {
          positionCallback(msg, topic, robot_name);
        });
    subscribers_.push_back(subscriber);
  }
}
void Planner::positionCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg, const std::string& topic,
    const std::string& robot_name) {  // Extract the position and orientation
  //                                     // information from the Odometry
  //                                     message
  // geometry_msgs::msg::Point position = msg->pose.pose.position;
  // geometry_msgs::msg::Quaternion orientation = msg->pose.pose.orientation;

  // // Update the internal state variables with the received position
  // information unsigned short robot_index = getRobotIndex(robot_name); if
  // (robot_index < positions_.size()) {
  //   positions_[robot_index].x = position.x;
  //   positions_[robot_index].y = position.y;
  //   positions_[robot_index].z = position.z;
  //   orientations_[robot_index] = orientation;
  // }

  // // Publish a message using the class publisher
  // // Example: Publishing a Twist message with zero linear and angular
  // velocities auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
  // robot_publishers_[robot_index]->publish(*twist_msg);
}
// unsigned short Planner::getRobotIndex(const std::string& robot_name) const {
//   for (size_t i = 0; i < robots_.size(); ++i) {
//     if (robots_[i] == robot_name) {
//       return i;
//     }
//   }
//   return std::numeric_limits<unsigned short>::max();  // Return an invalid
//   index
//                                                       // if the robot name is
//                                                       // not found
// }
void Planner::createAllPublishers() {
  for (const auto& robot_name : robots_) {
    std::string topic = "/" + robot_name + "/cmd_vel";
    auto publisher = create_publisher<geometry_msgs::msg::Twist>(topic, 10);
    robot_publishers_.push_back(publisher);
  }

  std::string alarm_topic = "/planning_alarm";
  alarm_publisher_ = create_publisher<std_msgs::msg::String>(alarm_topic, 10);
}
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