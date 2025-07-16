cmake_minimum_required(VERSION 3.8)
project(ros2_pure_pursuit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(a_star_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(pure_pursuit_controller_node src/pure_pursuit_controller_node.cpp)
ament_target_dependencies(pure_pursuit_controller_node rclcpp nav_msgs geometry_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros)
target_include_directories(pure_pursuit_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(static_path_publisher_node src/static_path_publisher_node.cpp)
ament_target_dependencies(static_path_publisher_node rclcpp nav_msgs geometry_msgs)
target_include_directories(static_path_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(a_star_path_client_node src/a_star_path_client_node.cpp)
ament_target_dependencies(a_star_path_client_node rclcpp rclcpp_action nav_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_ros a_star_interfaces)
target_include_directories(a_star_path_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS pure_pursuit_controller_node static_path_publisher_node a_star_path_client_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz/
)

install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs/
)

ament_package()
