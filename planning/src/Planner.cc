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
  obstacles_.push_back(std::make_tuple(500, 500));  // Dummy obstacle
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
  //This will store the coordinates of the waypoints for the goal for each robot
  std::vector<geometry_msgs::msg::Point> cbs_last_waypoints;

  for (size_t robot_index = 0; robot_index < target_waypoints_.size();
       ++robot_index) {
    cbs_last_waypoints.emplace_back(target_waypoints_[robot_index].back());
  }

  for (size_t robot_index = 0; robot_index < robots_.size(); ++robot_index) {
    auto current_position = positions_[robot_index];

   //The odom topic gives coordinates in real frame, but cbs goal is in cbs coordinates
   //these have to be converted before finding out the distance
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
  int number_of_robots_in_waypoints = 0;

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

  if (number_of_robots_in_waypoints == static_cast<int>(robots_.size())) {
    RCLCPP_INFO(get_logger(), "Robots Arrived in waypoint: " +
                                  std::to_string(cbs_time_schedule_));
    return true;
  } else {
    return false;
  }
}

const geometry_msgs::msg::Point& Planner::getNextTargetWaypoints(
    const int robot_index) {
  if (static_cast<int>(target_waypoints_[robot_index].size()) >
      cbs_time_schedule_) {
    return target_waypoints_[robot_index][cbs_time_schedule_];
  } else {
    return target_waypoints_[robot_index][max_cbs_times_[robot_index]];
  }
}

int Planner::getRobotIndex(const std::string& robot_name) const {
  for (size_t i = 0; i < robots_.size(); ++i) {
    if (robots_[i] == robot_name) {
      return i;
    }
  }
  return std::numeric_limits<int>::max();  // Return an invalid index
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
int Planner::countRobotTopics() {
  auto topic_list = this->get_topic_names_and_types();
  int robot_count = 0;

  for (const auto& item : topic_list) {
    if (item.first.find("/robot") == 0 &&
        item.first.find("/cmd_vel") != std::string::npos) {
      robot_count++;
    }
  }

  return robot_count;
}
void Planner::updateGoal(int robot_id,
                         const geometry_msgs::msg::Point& new_goal) {
  final_goal_[robot_id] = new_goal;
  RCLCPP_INFO(get_logger(), "Updated Goal for robot-%d: [%f, %f]", robot_id,
              new_goal.x, new_goal.y);
}

void Planner::driveRobotstoCbsWaypoints() {}
void Planner::callCbsPlanner() {
  verifyInitialRobotPositions();
  haltRobots();
  RCLCPP_INFO(get_logger(), "Conflict Based Search Planning");
  RCLCPP_INFO(get_logger(), "Searching for solution...");
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
void Planner::commandRobot(int robot_index,
                           const geometry_msgs::msg::Point& target_waypoint,
                           double current_orientation) {
  double d = 0.1;  // Virtual point outside robot center (avoid mathematical
                   // errors in the feedback linearization controller)
  geometry_msgs::msg::Point current_position = positions_[robot_index];
  geometry_msgs::msg::Twist cmd_vel;
  try {
    // Feedback linearization controller, get linear and angular desired
    // velocities from desired X and Y Points are shifted (SHIFT_MAP) by a fixed
    // amount so that they are positive and are shrinked/inflated
    // (DISCRETIZATION) to allow the use of CBS that only accepts positive
    // integers
    double target_x_scaled = (target_waypoint.x - shift_map_) / discretization_;
    double target_y_scaled = (target_waypoint.y - shift_map_) / discretization_;
    double error_x = target_x_scaled - current_position.x;
    double error_y = target_y_scaled - current_position.y;
    cmd_vel.linear.x =
        (error_x)*cos(current_orientation) + (error_y)*sin(current_orientation);
    cmd_vel.angular.z = -(error_x)*sin(current_orientation) / d +
                        (error_y)*cos(current_orientation) / d;
    cmd_vel.linear.x = Kp_ * cmd_vel.linear.x;
    cmd_vel.angular.z = Kp_ * cmd_vel.angular.z;
    // Limit the velocities
    cmd_vel.linear.x =
        std::min(cmd_vel.linear.x, static_cast<double>(max_linear_velocity_));
    cmd_vel.angular.z =
        std::min(cmd_vel.angular.z, static_cast<double>(max_angular_velocity_));
    auto publisher =
        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::Twist>>(
            robot_publishers_[robot_index]);
    publisher->publish(cmd_vel);
  } catch (const std::exception& e) {
    std::string error_msg =
        "Error occurred during computation: " + std::string(e.what());
    RCLCPP_ERROR(get_logger(), error_msg);
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
    if (position_received_ ==
        static_cast<int>(robots_.size())) {  // All positions received
      RCLCPP_INFO(get_logger(), "All positions received");
      first_time_planning_ = false;
      return true;
    }
  }
  return false;
}
void Planner::resolveGoalConflicts() {
  bool conflict = true;
  while (conflict) {
    conflict = false;
    for (size_t robot_i = 0; robot_i < final_goal_.size(); ++robot_i) {
      for (size_t robot_j = 0; robot_j < final_goal_.size(); ++robot_j) {
        if (robot_i > robot_j && final_goal_[robot_i] == final_goal_[robot_j]) {
          RCLCPP_INFO(get_logger(),
                      "Same goal for robot-" + std::to_string(robot_i) +
                          " and robot-" + std::to_string(robot_j));
          geometry_msgs::msg::Point new_goal =
              generateNewGoal(final_goal_[robot_i]);
          updateGoal(robot_i, new_goal);
          conflict = true;
        }
      }
    }
  }
}

geometry_msgs::msg::Point Planner::generateNewGoal(
    const geometry_msgs::msg::Point& old_goal) {
  geometry_msgs::msg::Point new_goal;
  new_goal.x = old_goal.x + std::rand() % 3 - 1;
  new_goal.y = old_goal.y + std::rand() % 3 - 1;
  new_goal.z = 0.0;
  return new_goal;
}
void Planner::resolveObstacleConflicts() {
  bool conflict = true;
  while (conflict) {
    conflict = false;
    for (size_t robot_i = 0; robot_i < final_goal_.size(); ++robot_i) {
      for (const auto& obstacle_i : obstacles_) {
        if (static_cast<int>(final_goal_[robot_i].x + shift_map_) ==
                std::get<0>(obstacle_i) &&
            static_cast<int>(final_goal_[robot_i].y + shift_map_) ==
                std::get<1>(obstacle_i)) {
          RCLCPP_INFO(get_logger(),
                      "Robot-" + std::to_string(robot_i) + " goal (" +
                          std::to_string(final_goal_[robot_i].x) + ", " +
                          std::to_string(final_goal_[robot_i].y) +
                          ") is inside obstacle-(" +
                          std::to_string(std::get<0>(obstacle_i)) + ", " +
                          std::to_string(std::get<1>(obstacle_i)) + ")");
          geometry_msgs::msg::Point new_goal =
              generateNewGoal(final_goal_[robot_i]);
          updateGoal(robot_i, new_goal);
          conflict = true;
        }
      }
    }
  }
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