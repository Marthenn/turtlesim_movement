cmake_minimum_required(VERSION 3.8)
project(movement)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(poseSubscriber src/PoseSubscriber.cpp)
ament_target_dependencies(poseSubscriber rclcpp turtlesim)

install(
    TARGETS poseSubscriber
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
