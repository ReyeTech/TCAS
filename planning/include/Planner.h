#ifndef PLANNER_H
#define PLANNER_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "cbs.h"

namespace TCAS {
class Planner : public rclcpp::Node {
 public:
  Planner();

 private:
  void init();
  void createAllSubscribers();
  void createAllPublishers();
  unsigned short countRobotTopics();

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
};
}  // namespace TCAS
#endif