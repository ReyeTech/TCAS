#ifndef PLANNER_H
#define PLANNER_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tuple>
#include <vector>

#include "CBS.h"

namespace TCAS {
class Planner : public rclcpp::Node {
 public:
  Planner();

 private:
  // Initialize the Planner object
  void init();
  // Create one subscriber per robot for /odom
  void createAllSubscribers();
  // Create one publisher per robot for /cmd_vel and a common alarm topic
  void createAllPublishers();
  // Subsriber Position callback function- Recieves position of each robot and
  // call the controller
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

  // Getting the robot index for positionCallback function
  int getRobotIndex(const std::string &) const;
  // Count number of robot topics to know the number of robots avoiding
  // hardcoding it here. Returns a short
  int countRobotTopics();
  float getDistance(const float dx, const float dy);
  // Main driving function to go through cbs waypoints. Drive robots to the
  // waypoints that are solution of the CBS planner.
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
   * bot goals coinciding or the goal being inside an obstacle
   */
  geometry_msgs::msg::Point generateNewGoal(const geometry_msgs::msg::Point &);
  /**
   * Assign new x, y coordinates to the goal position
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
 * Set velocity for the robots, based on the distance between target waypoint and the current position
*/
  void commandRobot(int, const geometry_msgs::msg::Point &, double);
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

  int number_of_robots_;
  std::vector<std::string> robots_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers_;
  std::vector<rclcpp::PublisherBase::SharedPtr> robot_publishers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alarm_publisher_;
  std::vector<std::vector<geometry_msgs::msg::Point>> target_waypoints_;
  std::vector<geometry_msgs::msg::Point> positions_;
  std::vector<float> orientations_;  // stores the yaw value
  std::vector<int> max_cbs_times_;
  std::vector<float> distance_to_target_;
  std::vector<geometry_msgs::msg::Point> final_goal_;
  int position_received_;
  bool first_time_planning_;
  int cbs_time_schedule_;
  int number_of_successfully_executed_plans_;
  std::vector<std::tuple<int, int>>
      obstacles_;  //(x,y) coordinates of the obstacles
};
}  // namespace TCAS
#endif