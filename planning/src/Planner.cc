#include "Planner.h"
#define NUMBER_OF_ROBOTS 3
namespace TCAS {

Planner::Planner() : Node("robot_controller") {
  init();
  updateObstacleLocations();
  readCustomGoals();
  createAllSubscribers();
  createAllPublishers();
}

void Planner::init() {
  RCLCPP_INFO(get_logger(), "Planner init has been called");
  // populating number of robots and robots_ vector
  number_of_robots_ = 0;
  while (number_of_robots_ < NUMBER_OF_ROBOTS) {
    number_of_robots_ = countRobotTopics();
  }
  RCLCPP_INFO(get_logger(), "Number of robots is %d", number_of_robots_);
  robots_.resize(number_of_robots_);
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
void Planner::createAllSubscribers() {
  for (const auto& robot_name : robots_) {
    std::string topic = "/" + robot_name + "/odom";
    auto subscriber = create_subscription<nav_msgs::msg::Odometry>(
        topic, 10, [this, topic](const nav_msgs::msg::Odometry::SharedPtr msg) {
          positionCallback(msg, topic);
        });
    subscriber_odom_.push_back(subscriber);
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
    // RCLCPP_INFO(get_logger(), "Position of robot-%u: [x=%.2f, y=%.2f,
    // z=%.2f]",
    //             robot_index, positions_[robot_index].x,
    //             positions_[robot_index].y, positions_[robot_index].z);

    // Compute a new control input for every update in position
    driveRobotstoCbsWaypoints();
  }
}
void Planner::readCustomGoals() {
  std::string input_filename = "scripts/params/custom_goals.yaml";
  std::string filename = getFullFilename(input_filename);
  std::ifstream file(filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file: %s", filename.c_str());
    return;
  }

  YAML::Node data = YAML::Load(file);
  YAML::Node data_robots = data["robots"];
  for (std::size_t robot_index = 0;
       robot_index < static_cast<std::size_t>(number_of_robots_);
       ++robot_index) {
    if (robot_index < data_robots.size()) {
      YAML::Node robot_data = data_robots[robot_index];
      YAML::Node goal_node = robot_data["goal"];
      if (goal_node.IsSequence() && goal_node.size() >= 2) {
        double goal_x = std::round(goal_node[0].as<double>());
        double goal_y = std::round(goal_node[1].as<double>());
        RCLCPP_INFO(get_logger(), "goal : x %.2f y %.2f", goal_x, goal_y);
        final_goal_[robot_index].x = goal_x;
        final_goal_[robot_index].y = goal_y;
      }
    }
  }

  resolveGoalConflicts();
  resolveObstacleConflicts();
}

void Planner::goalCallback(const geometry_msgs::msg::Point::SharedPtr msg,
                           const std::string& topic) {}

bool Planner::allRobotsArrivedCBSFinalWaypoint() {}

bool Planner::allRobotsArrivedInWaypoints() {}

const geometry_msgs::msg::Point& Planner::getNextTargetWaypoints(
    const int robot_index) {}

int Planner::getRobotIndex(const std::string& robot_name) const {}
void Planner::createAllPublishers() {
  for (const auto& robot_name : robots_) {
    std::string topic = "/" + robot_name + "/cmd_vel";
    auto publisher = create_publisher<geometry_msgs::msg::Twist>(topic, 10);
    robot_publisher_velocity_.push_back(publisher);
  }
  std::string alarm_topic = "/planning_alarm";
  alarm_publisher_ = create_publisher<std_msgs::msg::String>(alarm_topic, 10);
}

bool Planner::updateObstacleLocations() {  // Load map and yaml file
  const std::string map_filename = "/map/my_map.pgm";
  const std::string map_file_path = getFullFilename(map_filename);
  const std::string yaml_filename = "/map/my_map.yaml";
  const std::string yaml_file_path = getFullFilename(yaml_filename);

  RCLCPP_ERROR(get_logger(), "Map file_path is %s\n", map_file_path.c_str());
  // Read in the occupancy grid map from a PGM file
  std::ifstream map_file(map_file_path, std::ios::binary);
  if (!map_file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Could not open map file");
    return 0;
  }
  std::string line;
  std::getline(map_file, line);
  if (line != "P5") {
    RCLCPP_ERROR(get_logger(), "Invalid PGM file format");
    return 0;
  }
  std::getline(map_file, line);
  while (line[0] == '#') {
    std::getline(map_file, line);
  }
  int width, height;
  std::istringstream(line) >> width >> height;
  std::getline(map_file, line);
  int max_value;
  std::istringstream(line) >> max_value;

  std::vector<std::vector<Cell>> map(height, std::vector<Cell>(width));

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      Cell cell;
      uint8_t value = map_file.get();
      if (value == 0) {
        cell.obstacle = false;
      } else {
        cell.obstacle = true;
      }
      cell.x = x;
      cell.y = y;
      map[y][x] = cell;
    }
  }

  map_file.close();

  // Read in the origin and resolution data from a YAML file
  double origin_x, origin_y, resolution;
  YAML::Node root = YAML::LoadFile(yaml_file_path);
  if (!root["resolution"] || !root["origin"]) {
    RCLCPP_ERROR(get_logger(),
                 "Missing origin or resolution data in YAML file");
    return 0;
  }
  resolution = root["resolution"].as<double>();
  YAML::Node origin_node = root["origin"];
  if (origin_node.Type() == YAML::NodeType::Sequence) {
    origin_x = origin_node[0].as<double>();
    origin_y = origin_node[1].as<double>();
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid origin data in YAML file");
    return 0;
  }

  // Convert the obstacle positions to world frame
  for (size_t y = 0; y < map.size(); y++) {
    for (size_t x = 0; x < map[y].size(); x++) {
      if (map[y][x].obstacle) {
        // To do - change to float to improve accuracy
        int x_world = origin_x + x * resolution;
        int y_world = origin_y + y * resolution;
        obstacles_.emplace_back(x_world, y_world);
      }
    }
  }
  RCLCPP_INFO(get_logger(), "Number of obstacles added : %d",
              obstacles_.size());
  return 1;
}

void Planner::updateGoal(int robot_id,
                         const geometry_msgs::msg::Point& new_goal) {
  final_goal_[robot_id] = new_goal;
  RCLCPP_INFO(get_logger(), "Updated Goal for robot-%d: [%f, %f]", robot_id,
              new_goal.x, new_goal.y);
}

void Planner::driveRobotstoCbsWaypoints() {
  if (allPositionsReceived()) {
    callCbsPlanner();
  }
}
void Planner::callCbsPlanner() {
  verifyInitialRobotPositions();
  RCLCPP_INFO(get_logger(), "Halt robots being called");
  haltRobots();

  std::string inputFile;
  std::string outputFile;

  std::string inputFilename = "scripts/params/cbs_input.yaml";
  inputFile = getFullFilename(inputFilename);
  std::string outputFilename = "scripts/params/cbs_output.yaml";
  outputFile = getFullFilename(outputFilename);
  // Check if the input file is not empty
  std::ifstream inputCheckFile(inputFile);
  if (inputCheckFile.peek() != std::ifstream::traits_type::eof()) {
    // Clear the input file
    std::ofstream inputClearFile(inputFile);
    if (inputClearFile.is_open()) {
      inputClearFile.close();
      RCLCPP_INFO(get_logger(), "Input file cleared");
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to clear input file");
      return;
    }
  }
  inputCheckFile.close();

  // Check if the output file is not empty
  std::ifstream outputCheckFile(outputFile);
  if (outputCheckFile.peek() != std::ifstream::traits_type::eof()) {
    // Clear the output file
    std::ofstream outputClearFile(outputFile);
    if (outputClearFile.is_open()) {
      outputClearFile.close();
      RCLCPP_INFO(get_logger(), "Output file cleared");
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to clear output file");
      return;
    }
  }
  outputCheckFile.close();

  writeDataToYaml(inputFile);
  std::this_thread::sleep_for(std::chrono::seconds(1));
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
      if (robot_index < robot_publisher_velocity_.size()) {
        robot_publisher_velocity_[robot_index]->publish(cmd_vel);
      } else {
        RCLCPP_ERROR(get_logger(), "Invalid publisher index for robot %zu",
                     robot_index);
      }
    } catch (const std::exception& e) {
      std::string error_msg = "Error halting robots: " + std::string(e.what());
      RCLCPP_ERROR(get_logger(), error_msg);
    }
  }
}
void Planner::commandRobot(int robot_index,
                           const geometry_msgs::msg::Point& target_waypoint,
                           double current_orientation) {}

bool Planner::allPositionsReceived() {
  if (first_time_planning_) {
    position_received_ = 0;
    for (size_t robot_index = 0; robot_index < robots_.size(); ++robot_index) {
      auto current_position = positions_[robot_index];
      if (current_position.x != 0 && current_position.y != 0) {
        position_received_++;
        RCLCPP_INFO(get_logger(), "No of positions recieved: %d",
                    position_received_);
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
  RCLCPP_INFO(get_logger(), "Exiting verify initial robot positions");
}

float Planner::getDistance(const float dx, const float dy) {
  return sqrt(dx * dx + dy * dy);
}
std::string Planner::getFullFilename(const std::string& paramFilename) {
  std::string packagePath =
      ament_index_cpp::get_package_share_directory("planning");
  std::string substring = "install/planning/share/";
  std::string filename = packagePath + "/" + paramFilename;
  size_t pos = filename.find(substring);
  if (pos != std::string::npos) {
    filename.replace(pos, substring.length(), "");
  }
  return filename;
}

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