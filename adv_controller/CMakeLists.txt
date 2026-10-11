cmake_minimum_required(VERSION 3.8)
project(adv_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(logistic_msg REQUIRED)
# find_package(algorithm REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(nav2_clint src/nav2_action_clinet.cpp)
ament_target_dependencies(nav2_clint rclcpp rclcpp_action nav2_msgs tf2_geometry_msgs geometry_msgs tf2)

add_executable(get_in_controller src/get_in_controller.cpp)
ament_target_dependencies(get_in_controller rclcpp geometry_msgs sensor_msgs logistic_msg std_msgs message_filters)

add_executable(orgnizer src/orgnizer_con.cpp)
ament_target_dependencies(orgnizer rclcpp rclcpp_action geometry_msgs logistic_msg std_msgs tf2 tf2_geometry_msgs)

add_executable(adv_getin_controller src/adv_getin_controller.cpp)
ament_target_dependencies(adv_getin_controller rclcpp geometry_msgs sensor_msgs logistic_msg std_msgs)


install(TARGETS
  nav2_clint
  get_in_controller
  orgnizer
  adv_getin_controller
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
