cmake_minimum_required(VERSION 3.5)
project(fusion_matcher)

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

#add_compile_options(--save-temps)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(erae_std_msgs REQUIRED)
find_package(erae_sensor_msgs REQUIRED)
find_package(erae_perception_msgs REQUIRED)
find_package(erae_fusion_msgs REQUIRED)
find_package(erae_vehicle_msgs REQUIRED)
find_package(erae_diag_msgs REQUIRED)

find_package(ebase_node REQUIRED)

include_directories(include)

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

add_executable(match_node
		src/main.cpp
		src/match_node.cpp
    src/fusion_algorithm/fusion_algorithm.cpp
    #src/fusion_algorithm/relative_speed_fusion.cpp
    src/fusion_algorithm/fusion_algorithm_factory.cpp
    #src/fusion_algorithm/vehicle_speed_fusion.cpp
    src/fusion_algorithm/simple_fusion.cpp
    src/fusion_id_manager.cpp
    src/parameter/match_parameter.cpp
    src/parameter/node_parameter.cpp
    src/radar_data.cpp
    src/radar_matching_command.cpp
    src/percept_data.cpp
    src/percept_matching_command
    src/fusion_data.cpp
    src/algorithm/percept_matching_algorithm.cpp
    src/algorithm/radar_matching_algorithm.cpp)
ament_target_dependencies(match_node rclcpp visualization_msgs OpenCV cv_bridge sensor_msgs erae_std_msgs erae_sensor_msgs std_msgs
  erae_perception_msgs erae_fusion_msgs erae_diag_msgs ebase_node erae_vehicle_msgs)

# Install
install(TARGETS
  match_node
	DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
