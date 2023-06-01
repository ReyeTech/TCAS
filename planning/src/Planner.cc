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
  target_waypoints_.resize(number_of_robots_);
  positions_.resize(number_of_robots_);
  orientations_.resize(number_of_robots_);
  distance_to_target_.resize(number_of_robots_);
  max_cbs_times_.resize(number_of_robots_);
  final_goal_.resize(number_of_robots_);

  for (int i = 0; i < number_of_robots_; ++i) {
    robots_[i] = "robot" + std::to_string(i);
    positions_[i] = geometry_msgs::msg::Point();
    orientations_[i] = 0.0;
    distance_to_target_[i] = 0.0;
    max_cbs_times_[i] = 0;
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
        topic, 10, [this, topic](const nav_msgs::msg::Odometry::SharedPtr msg) {
          positionCallback(msg, topic);
        });
    subscribers_.push_back(subscriber);
  }
}
void Planner::positionCallback(const nav_msgs::msg::Odometry::SharedPtr msg,
                               const std::string& topic) {
  std::regex robot_regex("/robot(\\d+)/odom");
  std::smatch match;
  if (std::regex_search(topic, match, robot_regex)) {
    std::string robot_index_str = match[1].str();
    unsigned int robot_index = std::stoi(robot_index_str);

    geometry_msgs::msg::Point position = msg->pose.pose.position;
    geometry_msgs::msg::Quaternion orientation = msg->pose.pose.orientation;
    tf2::Quaternion quat(orientation.x, orientation.y, orientation.z,
                         orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    positions_[robot_index].x = position.x;
    positions_[robot_index].y = position.y;
    positions_[robot_index].z = position.z;
    orientations_[robot_index] = yaw;

    // Compute a new control input for every update in position
    driveRobotstoCbsWaypoints();
  }
}

bool Planner::allRobotsArrivedCBSFinalWaypoint() {
  int number_of_robots_in_target = 0;
  std::vector<geometry_msgs::msg::Point> cbs_last_waypoints;

  for (size_t robot_index = 0; robot_index < target_waypoints_.size();
       ++robot_index) {
    cbs_last_waypoints.emplace_back(target_waypoints_[robot_index].back());
  }

  for (size_t robot_index = 0; robot_index < robots_.size(); ++robot_index) {
    auto current_position = positions_[robot_index];

    // Points are shifted (shift_map_) by a fixed amount so that they are
    // positive and are shrinked/inflated (discretization_) to allow the use of
    // CBS that only accepts positive integers
    distance_to_target_[robot_index] = getDistance(
        (cbs_last_waypoints[robot_index].x - shift_map_) / discretization_ -
            current_position.x,
        (cbs_last_waypoints[robot_index].y - shift_map_) / discretization_ -
            current_position.y);

    if (distance_to_target_[robot_index] < threshold_bot_on_target_) {
      number_of_robots_in_target++;
    }
  }

  // All robots on last waypoint
  if (static_cast<size_t>(number_of_robots_in_target) == robots_.size()) {
    haltRobots();
    number_of_successfully_executed_plans_++;
    RCLCPP_WARN(get_logger(),
                "Number of successfully executed plans: " +
                    std::to_string(number_of_successfully_executed_plans_));
    cbs_time_schedule_ = 1;  // Plan again
    return true;
  } else {
    return false;
  }
}

bool Planner::allRobotsArrivedInWaypoints() {
  size_t number_of_robots_in_waypoints = 0;

  for (size_t robot_index = 0; robot_index < robots_.size(); ++robot_index) {
    auto current_position = positions_[robot_index];
    auto target_waypoint = getNextTargetWaypoints(robot_index);
    // Points are shifted (SHIFT_MAP) by a fixed amount so that they are
    // positive and are shrinked/inflated (DISCRETIZATION) to allow the use of
    // CBS that only accepts positive integers
    distance_to_target_[robot_index] = getDistance(
        (target_waypoint.x - shift_map_) / discretization_ - current_position.x,
        (target_waypoint.y - shift_map_) / discretization_ -
            current_position.y);

    if (distance_to_target_[robot_index] < threshold_bot_on_target_) {
      number_of_robots_in_waypoints++;
    }
  }

  if (number_of_robots_in_waypoints == robots_.size()) {
    RCLCPP_INFO(get_logger(), "Robots Arrived in waypoint: " +
                                  std::to_string(cbs_time_schedule_));
    return true;
  } else {
    return false;
  }
}

const geometry_msgs::msg::Point& Planner::getNextTargetWaypoints(
    const unsigned short robot_index) {
  if (target_waypoints_[robot_index].size() > cbs_time_schedule_) {
    return target_waypoints_[robot_index][cbs_time_schedule_];
  } else {
    return target_waypoints_[robot_index][max_cbs_times_[robot_index]];
  }
}

unsigned short Planner::getRobotIndex(const std::string& robot_name) const {
  for (size_t i = 0; i < robots_.size(); ++i) {
    if (robots_[i] == robot_name) {
      return i;
    }
  }
  return std::numeric_limits<unsigned short>::max();  // Return an invalid index
  // if the robot name is
  // not found
}
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

void Planner::driveRobotstoCbsWaypoints() {}
void Planner::callCbsPlanner() {
  verifyInitialRobotPositions();
  haltRobots();
}

void Planner::haltRobots() {
  geometry_msgs::msg::Twist cmd_vel;
  for (size_t robot_index = 0; robot_index < robots_.size(); ++robot_index) {
    try {
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.x = 0.0;
      cmd_vel.angular.y = 0.0;
      cmd_vel.angular.z = 0.0;
      auto publisher = std::static_pointer_cast<
          rclcpp::Publisher<geometry_msgs::msg::Twist>>(
          robot_publishers_[robot_index]);
      publisher->publish(cmd_vel);
    } catch (const std::exception& e) {
      std::string error_msg = "Error halting robots: " + std::string(e.what());
      RCLCPP_ERROR(get_logger(), error_msg);
    }
  }
}

bool Planner::allPositionsReceived() {
  if (first_time_planning_) {
    position_received_ = 0;
    for (size_t robot_index = 0; robot_index < robots_.size(); ++robot_index) {
      auto current_position = positions_[robot_index];
      if (current_position.x != 0 && current_position.y != 0) {
        position_received_++;
      }
    }
    if (position_received_ == robots_.size()) {  // All positions received
      RCLCPP_INFO(get_logger(), "All positions received");
      first_time_planning_ = false;
      return true;
    }
  }
  return false;
}

void Planner::verifyInitialRobotPositions() {
  for (size_t robot_i = 0; robot_i < robots_.size(); ++robot_i) {
    for (size_t robot_j = 0; robot_j < robots_.size(); ++robot_j) {
      if (robot_i != robot_j) {
        double dist =
            getDistance(positions_[robot_i].x - positions_[robot_j].x,
                        positions_[robot_i].y - positions_[robot_j].y);
        if (dist < 0.8 * (1 / discretization_)) {
          RCLCPP_INFO(get_logger(),
                      "Robots are too close, CBS will not find a solution");
        }
      }
    }
  }
}

float Planner::getDistance(const float dx, const float dy) {
  return sqrt(dx * dx + dy * dy);
}

}  // namespace TCAS
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TCAS::Planner>());
  rclcpp::shutdown();
  return 0;
}