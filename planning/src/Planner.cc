#include "Planner.h"
#define NUMBER_OF_ROBOTS 3
namespace TCAS {

Planner::Planner() : Node("robot_controller") {
  init();
  // updateObstacleLocations();
  readObstacleParams();
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
  // obstacles_.push_back(std::make_tuple(500, 500));  // Dummy obstacle
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

bool Planner::allRobotsArrivedCBSFinalWaypoint() {
  int number_of_robots_in_target = 0;
  // This will store the coordinates of the waypoints for the goal for each
  // robot
  std::vector<geometry_msgs::msg::Point> cbs_last_waypoints;

  for (size_t robot_index = 0; robot_index < target_waypoints_.size();
       ++robot_index) {
    cbs_last_waypoints.emplace_back(target_waypoints_[robot_index].back());
  }

  for (size_t robot_index = 0; robot_index < robots_.size(); ++robot_index) {
    auto current_position = positions_[robot_index];

    // The odom topic gives coordinates in real frame, but cbs goal is in cbs
    // coordinates these have to be converted before finding out the distance
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
void Planner::readObstacleParams() {
  std::string inputFilename = "scripts/params/custom_obstacles.yaml";
  std::string filename = getFullFilename(inputFilename);

  YAML::Node data = YAML::LoadFile(filename);

  const YAML::Node& obstaclesData = data["obstacles"];
  for (const auto& obstacleData : obstaclesData) {
    const YAML::Node& obstacle = obstacleData["obstacle"];
    int x = static_cast<int>(obstacle[0].as<double>() * discretization_ +
                             shift_map_);
    int y = static_cast<int>(obstacle[1].as<double>() * discretization_ +
                             shift_map_);
    obstacles_.emplace_back(std::make_tuple(x, y));
  }
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
  if (!first_time_planning_) {
    if (allRobotsArrivedCBSFinalWaypoint()) {
      RCLCPP_WARN(get_logger(), "Plan executed successfully!");
      std::exit(0);

    } else {
      if (allRobotsArrivedInWaypoints()) {
        if (cbs_time_schedule_ <
            *std::max_element(max_cbs_times_.begin(), max_cbs_times_.end())) {
          cbs_time_schedule_++;  // Next time in CBS schedule
          RCLCPP_INFO(
              get_logger(), "Going to waypoint: %d | of total: %d",
              cbs_time_schedule_,
              *std::max_element(max_cbs_times_.begin(), max_cbs_times_.end()));
        }
      }
      for (size_t robotIndex = 0; robotIndex < robots_.size(); ++robotIndex) {
        geometry_msgs::msg::Point targetWaypoint =
            getNextTargetWaypoints(robotIndex);
        double currentOrientation = orientations_[robotIndex];
        // Validate currentOrientation
        if (!std::isfinite(currentOrientation)) {
          RCLCPP_ERROR(get_logger(), "Invalid currentOrientation: %f",
                       currentOrientation);
          continue;
        }
        commandRobot(robotIndex, targetWaypoint, currentOrientation);
      }
    }
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
  cbs_.executeCbs(inputFile, outputFile);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(get_logger(), "Cbs execution finished..");
  getDataFromYaml(outputFile);
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
                           double current_orientation) {
  double d = 0.1;  // Virtual point outside robot center (avoid mathematical
                   // errors in the feedback linearization controller)
  geometry_msgs::msg::Point current_position = positions_[robot_index];
  geometry_msgs::msg::Twist cmd_vel;
  RCLCPP_INFO(get_logger(), "entered command robot function");
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
    // auto publisher =
    //     std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::Twist>>(
    //         robot_publisher_waypoints_[robot_index]);
    // publisher->publish(cmd_vel);
    robot_publisher_velocity_[robot_index]->publish(cmd_vel);
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

void Planner::writeDataToYaml(std::string& filename) {
  std::string inputFilename = "scripts/params/cbs_input.yaml";
  filename = getFullFilename(inputFilename);

  YAML::Node yamlData;
  YAML::Node robotsData;
  YAML::Node mapData;
  YAML::Node obstaclesData;

  for (size_t robotIndex = 0; robotIndex < robots_.size(); ++robotIndex) {
    int startX = static_cast<int>(
        std::round(positions_[robotIndex].x * discretization_));
    int startY = static_cast<int>(
        std::round(positions_[robotIndex].y * discretization_));
    int goalX = static_cast<int>(std::round(final_goal_[robotIndex].x));
    int goalY = static_cast<int>(std::round(final_goal_[robotIndex].y));

    YAML::Node agentData;
    agentData["start"] = YAML::Node(YAML::NodeType::Sequence);
    agentData["start"].push_back(startX + shift_map_);
    agentData["start"].push_back(startY + shift_map_);
    agentData["goal"] = YAML::Node(YAML::NodeType::Sequence);
    agentData["goal"].push_back(goalX + shift_map_);
    agentData["goal"].push_back(goalY + shift_map_);
    agentData["name"] = robots_[robotIndex];

    robotsData.push_back(agentData);
  }

  for (const auto& obstacle : obstacles_) {
    YAML::Node obstacleData;
    // obstacleData.push_back(
    //     static_cast<int>(std::get<0>(obstacle) * discretization_ +
    //     shift_map_));
    // obstacleData.push_back(
    //     static_cast<int>(std::get<1>(obstacle) * discretization_ +
    //     shift_map_));
    // if readObstacleParams() is used
    obstacleData.push_back(std::get<0>(obstacle));
    obstacleData.push_back(std::get<1>(obstacle));
    obstaclesData.push_back(obstacleData);
  }

  mapData["dimensions"] = YAML::Node(YAML::NodeType::Sequence);
  mapData["dimensions"].push_back(cbs_map_dimension_);
  mapData["dimensions"].push_back(cbs_map_dimension_);
  mapData["obstacles"] = obstaclesData;

  yamlData["robots"] = robotsData;
  yamlData["map"] = mapData;
  RCLCPP_INFO(get_logger(), "Writing data to cbs_input---check the file");
  std::ofstream file(filename);
  if (file.is_open()) {
    file << yamlData;
    file.close();
  } else {
    std::cerr << "Failed to open file: " << filename << std::endl;
  }
}
void Planner::getDataFromYaml(std::string& filename) {
  std::string outputFilename = "scripts/params/cbs_output.yaml";
  filename = getFullFilename(outputFilename);

  try {
    YAML::Node data = YAML::LoadFile(filename);
    if (!data) {
      std_msgs::msg::String alarmMsg;
      alarmMsg.data = "Solution not found!";
      alarm_publisher_->publish(alarmMsg);
      RCLCPP_INFO(get_logger(), "Solution not found!");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      first_time_planning_ = true;
      RCLCPP_WARN(get_logger(), "Planning failed!");
      std::exit(0);
    } else {
      int status = 0;
      try {
        status = data["status"].as<int>();
      } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to read status from YAML: %s",
                     e.what());
        // Handle the error and return or throw an exception
        // ...
      }

      if (status == 0) {
        std_msgs::msg::String alarmMsg;
        alarmMsg.data = "Solution not found!";
        alarm_publisher_->publish(alarmMsg);
        RCLCPP_INFO(get_logger(), "Solution not found!");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        first_time_planning_ = true;
        RCLCPP_WARN(get_logger(), "Planning failed!");
        std::exit(0);
      } else {
        std_msgs::msg::String alarmMsg;
        alarmMsg.data = "Solution found!";
        alarm_publisher_->publish(alarmMsg);
        RCLCPP_INFO(get_logger(), "Solution found!");
      }
    }

    YAML::Node schedule = data["schedule"];
    target_waypoints_.resize(robots_.size());
    max_cbs_times_.resize(robots_.size());

    for (const auto& robot : schedule) {
      std::string robotName = robot.first.as<std::string>();
      int robotIndex = std::stoi(robotName.substr(5));

      for (const auto& wp : robot.second) {
        double xValue = 0.0;
        double yValue = 0.0;
        int tValue = 0;

        try {
          for (const auto& item : wp) {
            std::string key = item.first.as<std::string>();
            if (key == "x") {
              xValue = item.second.as<double>();
            } else if (key == "y") {
              yValue = item.second.as<double>();
            } else if (key == "t") {
              tValue = item.second.as<int>();
              max_cbs_times_[robotIndex] = tValue;
            }
          }
        } catch (const YAML::Exception& e) {
          RCLCPP_ERROR(get_logger(),
                       "Failed to read waypoint data from YAML: %s", e.what());
          // Handle the error and return or throw an exception
          // ...
        }

        geometry_msgs::msg::Point waypoint;
        waypoint.x = xValue;
        waypoint.y = yValue;
        waypoint.z = 0.01;
        target_waypoints_[robotIndex].push_back(waypoint);
      }
    }

    int maxWaypoints =
        *std::max_element(max_cbs_times_.begin(), max_cbs_times_.end());
    RCLCPP_INFO(get_logger(), "Max number of waypoints to execute: %d",
                maxWaypoints);
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to load YAML file: %s", e.what());
    // Handle the error and return or throw an exception
    // ...
  }

  // Rest of the code...
}

}  // namespace TCAS
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto plannerNode = std::make_shared<TCAS::Planner>();
  rclcpp::spin(plannerNode);
  rclcpp::shutdown();
  return 0;
}