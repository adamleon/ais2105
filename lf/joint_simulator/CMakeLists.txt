cmake_minimum_required(VERSION 3.8)
project(joint_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(joint_simulator_msg REQUIRED)

add_executable(pid_controller_node src/pid_controller_node.cpp)
ament_target_dependencies(pid_controller_node rclcpp std_msgs joint_simulator_msg)
target_include_directories(pid_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pid_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(joint_simulator_node src/joint_simulator_node.cpp)
ament_target_dependencies(joint_simulator_node rclcpp std_msgs joint_simulator_msg)
target_include_directories(joint_simulator_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(joint_simulator_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS joint_simulator_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
