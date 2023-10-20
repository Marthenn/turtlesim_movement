cmake_minimum_required(VERSION 3.8)
project(movement)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)

# add executables
add_executable(poseSubscriber src/subExec.cpp src/PoseSubscriber.cpp)
ament_target_dependencies(poseSubscriber rclcpp turtlesim)

add_executable(movePublisher src/pubExec.cpp src/MovementPublisher.cpp)
ament_target_dependencies(movePublisher rclcpp turtlesim geometry_msgs)

add_executable(thetaService src/srvExec.cpp src/ThetaServer.cpp)
ament_target_dependencies(thetaService rclcpp turtlesim interfaces)

# create install targets
install(
        TARGETS poseSubscriber movePublisher thetaService
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
