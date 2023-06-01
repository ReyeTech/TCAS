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
                        const std::string&);
  // Getting the robot index for positionCallback function
  unsigned short getRobotIndex(const std::string&) const;
  // Count number of robot topics to know the number of robots avoiding
  // hardcoding it here. Returns a short
  unsigned short countRobotTopics();
  // Main driving function to go through cbs waypoints. Drive robots to the waypoints that are solution of the CBS planner. 
  void driveRobotstoCbsWaypoints();

  bool custom_goals_ =
      false;  // True: read positions from /params/custom_goals.yaml
              // False: Random targets
  bool replan_ =
      true;  // True: Plan and execute continuosly False: Plan and execute once
  float threshold_bot_on_target_ =
      0.1;  // Threshold to consider that robot has reached a target
  unsigned short targets_random_pool_size_ =
      3;             // Size of target area in meters (Gazebo squares)
  float Kp_ = 0.15;  // Controller "proportional" gain
  unsigned short max_linear_velocity_ = 1;
  float max_angular_velocity_ = 0.2;
  unsigned short discretization_ =
      2;  // Discretization of map(1: one point per gazebo square,
          // 2 : 4 points per gazebo square)
  unsigned short shift_map_ =
      10 *
      discretization_;  // CBS only accepts positive integer values, all the
                        // values used by the CBS are shifted beforehand.10
                        // guarantees that if robots are inside gazebo
                        // dafault plane,all points sent to CBS are positive
  unsigned short cbs_map_dimension_ =
      20 * discretization_;  // This calculate the whole gazebo default area

  unsigned short number_of_robots_;
  std::vector<std::string> robots_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers_;
  std::vector<rclcpp::PublisherBase::SharedPtr> robot_publishers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alarm_publisher_;
  std::vector<geometry_msgs::msg::Point> positions_;
  std::vector<float> orientations_;  // stores the yaw value
  std::vector<float> distance_to_target_;
  std::vector<geometry_msgs::msg::Point> final_goal_;
  unsigned short position_received_;
  bool first_time_planning_;
  unsigned short cbs_time_schedule_;
  unsigned short number_of_successfully_executed_plans_;
  std::tuple<unsigned short, unsigned short> obstacles_;
};
}  // namespace TCAS
#endif