#ifndef PLANNER_H
#define PLANNER_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "CBS.h"

namespace TCAS {

struct Cell {
  bool obstacle;
  int x;
  int y;
};

class Planner : public rclcpp::Node {
 public:
  Planner();

 private:
  /**
   * Initializes the planner object
   */
  void init();
  /**
   * Create one subscriber per robot for /odom, need to subscribe to more topics
   * from here
   */
  void createAllSubscribers();
  /**
   * Create one publisher per robot for /cmd_vel and a common alarm topic
   */
  void createAllPublishers();
  /**
   * Subsriber Position callback function- Recieves position and orientation of
   * each robot and call the controller. Now this odom data is processed at some
   * frequency, the robot may or may not be at a waypoint
   */
  void positionCallback(const nav_msgs::msg::Odometry::SharedPtr,
                        const std::string &);

  /**
   * Check if all robots arrived in CBS last waypoint, i.e., the goal. Return
   * true only if all robots have arrived.
   */
  bool allRobotsArrivedCBSFinalWaypoint();

  /**
   *  Check if all robots arrived in a (mid) target. Return true only if all
   robots have arrived.

  */
  bool allRobotsArrivedInWaypoints();

  /**
   *  Get the next target waypoint in the CBS schedule, if it exists
   */
  const geometry_msgs::msg::Point &getNextTargetWaypoints(
      const int robot_index);

  /**
   * Getting the robot index for positionCallback function
   */

  int getRobotIndex(const std::string &) const;
  /**
   * Count number of robot topics. These topics are intialised by the multi
   * robot launch file which launches a gazebo world
   */

  int countRobotTopics();
  /**
   * Gets the eucledian distance
   */
  float getDistance(const float dx, const float dy);

  void driveRobotstoCbsWaypoints();
  /**
   *  Send zero to the robots if they are not supposed to move, avoid robots
   * wander around
   */
  void haltRobots();
  /**
   *  Check if the position of all robots have been received. Avoid errors of
   * computing planning before getting the actual position, calculating with
   * (0,0,0) initial published position
   */
  bool allPositionsReceived();
  /**
   * Check if two robots start in the same position (in relation to the
   * discretization)
   */
  void verifyInitialRobotPositions();

  void callCbsPlanner();
  /**
   * Generates a new goal close to the old one in case of any conflicts like two
   * bot goals coinciding or the goal being inside an obstacle. The output is
   * fed to void updateGoal function
   */
  geometry_msgs::msg::Point generateNewGoal(const geometry_msgs::msg::Point &);
  /**
   * Assign new x, y coordinates to the goal position. Used only in the case of
   * resolving conflicts
   */
  void updateGoal(int, const geometry_msgs::msg::Point &);
  /**
   * Checks if a goal given, lies within an obstacle and then finds a point
   * close by as the new goal
   */
  void resolveObstacleConflicts();
  /**
   * Checks if two bots have the same destination, assigns a point close to the
   * destination to one of them
   */
  void resolveGoalConflicts();
  /**
   * Set velocity for the robots, based on the distance between target waypoint
   * and the current position
   */
  void commandRobot(int, const geometry_msgs::msg::Point &, double);
  /**
   * Method to get the full file path
   */
  std::string getFullFilename(const std::string &paramFilename);
  /**
   * Creating the cbs_input.yaml file with data necessary for cbs algo
   */
  void writeDataToYaml(std::string &filename);
  /**
   * Read the output generated by the CBS algorithm
   */
  void getDataFromYaml(std::string &filename);
  /**
   * Create new goal points for the robots, this shall be done by reading from a
   * topic
   */
  void generateNewTargets(unsigned int, geometry_msgs::msg::Point);
  /**
   * read the goal (x,y) coordinates from a topic for each robot
   */
  void goalCallback(const geometry_msgs::msg::Point::SharedPtr,
                    const std::string &);
  /**
   * Get the map (x,y) coordinates of the obstacles from pgm file, into
   * obstacles_ in world frame
   */
  bool updateObstacleLocations();
  /**
   * Reads goals for obstacles
  */
  void readCustomGoals();
  CBSHelper cbs_;
  bool custom_goals_ =
      false;  // True: read positions from /params/custom_goals.yaml
              // False: Random targets
  bool replan_ =
      false;  // True: Plan and execute continuosly False: Plan and execute once
  float threshold_bot_on_target_ =
      0.1;  // Threshold to consider that robot has reached a target
  int targets_random_pool_size_ =
      3;             // Size of target area in meters (Gazebo squares)
  float Kp_ = 0.15;  // Controller "proportional" gain
  int max_linear_velocity_ = 1;
  float max_angular_velocity_ = 0.2;
  int discretization_ = 2;  // Discretization of map(1: one point per gazebo
                            // square, 2 : 4 points per gazebo square)
  int shift_map_ =
      10 *
      discretization_;  // CBS only accepts positive integer values, all the
                        // values used by the CBS are shifted beforehand.10
                        // guarantees that if robots are inside gazebo
                        // dafault plane,all points sent to CBS are positive
  int cbs_map_dimension_ =
      20 * discretization_;  // This calculate the whole gazebo default area

  /*
   * Number of robots in the gazebo world. Initialized in the multi_robot launch
   * file.
   */
  int number_of_robots_;
  /*
   * A vector containing the robot names: robot0, robot1 , etc . Equal in size
   * to the number of robots
   */
  std::vector<std::string> robots_;
  /**
   * Subscriber for goal positions
   */
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr>
      subscriber_goal_;
  /**
   * Subscriber for Odometry
   */
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
      subscriber_odom_;
  /**
   * Each robot can publish to the topics it likes. The size of the vector is
   * equal to the number of robots
   */
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr>
      robot_publisher_waypoints_;
  /**
   * Velocity publisher for each robot
   */
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr>
      robot_publisher_velocity_;
  /**
   * Creating a common publisher for the system, publishing to the planning
   * alarm. The topic is created in the create publisher code
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alarm_publisher_;
  /**
   * Stores the set of waypoints for each robot. Vector of Vectors
   */
  std::vector<std::vector<geometry_msgs::msg::Point>> target_waypoints_;
  /**
   * Stores the current position of each robot, using ROS2 point type (x,y)
   */
  std::vector<geometry_msgs::msg::Point> positions_;
  /**
   * Stores the yaw value for each of the robots
   */
  std::vector<float> orientations_;
  /**
   * Set of waypoints is different for different robots and time to reach goal
   * is also different for each of them. The goal reaching time is the max cbs
   * time
   */
  std::vector<int> max_cbs_times_;
  /**
   * Stores the distance remaining distance to the next waypoint for each robot
   */
  std::vector<float> distance_to_target_;
  /**
   * Stores the (x,y) coordinates of the goal for each robot
   */
  std::vector<geometry_msgs::msg::Point> final_goal_;
  /**
   * In case continuous planning is there, what is the current plan number
   */
  int position_received_;
  /**
   * True if plan number is 1
   */
  bool first_time_planning_;
  /**
   * Stores what stage or time corresponding to a waypoint number the bot has
   * reached
   */
  int cbs_time_schedule_;
  /**
   * Successfully completed plans
   */
  int number_of_successfully_executed_plans_;
  /**
   * stores the (x,y) coordinates of all the obstacles, in the form of an int
   */
  std::vector<std::tuple<int, int>> obstacles_;
  /**
   * Keeps track of whether a new goal is recieved on the topic /robot_name/goal
   */
  bool new_goal_received_;
};
}  // namespace TCAS
#endif