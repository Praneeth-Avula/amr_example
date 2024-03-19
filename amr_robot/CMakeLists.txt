cmake_minimum_required(VERSION 3.8)
project(amr_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(custom_msg_amr REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# Include yaml-cpp headers
include_directories(${YAML_CPP_INCLUDE_DIR})

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  custom_msg_amr
  yaml-cpp
  ament_cmake_gtest
  ${OTHER_DEPS}
)

include_directories(include)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  ament_add_gtest(robot_tests tests/robot_tests.cpp)
  ament_target_dependencies(robot_tests ${dependencies})
  target_link_libraries(robot_tests yaml-cpp)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
endif()

set(executable_name order_optimizer)

add_executable(${executable_name} src/order_optimizer.cpp)


ament_target_dependencies(${executable_name} ${dependencies})

target_link_libraries(${executable_name} yaml-cpp)



install (TARGETS
  order_optimizer
  robot_tests
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
