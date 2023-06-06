cmake_minimum_required(VERSION 3.5)
project(planning)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# for CBS
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

# Find and include Boost library for regular expressions
find_package(Boost REQUIRED COMPONENTS regex)
include_directories(${Boost_INCLUDE_DIRS})

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

#Include directory for the header files
include_directories(include)


# Install launch
install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS  
  scripts/planner.py
  scripts/cbs.py
  scripts/a_star.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(test_ros2 src/TestROS2.cc include/TestROS2.h)
ament_target_dependencies(test_ros2  rclcpp std_msgs nav_msgs)

install(TARGETS 
 test_ros2
 DESTINATION lib/${PROJECT_NAME}
)

# Add CBS library
add_library(cbs_lib
 src/CBS.cc 
 src/Astar.cc 
 include/CBS.h 
 include/Astar.h 
 include/Neighbor.h 
 include/PlanResult.h 
 include/Timer.h
)

target_link_libraries(cbs_lib
  ${Boost_LIBRARIES} 
  yaml-cpp
)

# Add executable for Planner_cpp
add_executable(Planner_cpp src/Planner.cc include/Planner.h)
ament_target_dependencies(Planner_cpp rclcpp std_msgs nav_msgs tf2 tf2_ros)
target_link_libraries(Planner_cpp ${Boost_LIBRARIES})

# Link CBS lib with executable planner
target_link_libraries(Planner_cpp cbs_lib)

install(TARGETS
  Planner_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/
  DESTINATION
    include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
