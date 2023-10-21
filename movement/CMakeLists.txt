cmake_minimum_required(VERSION 3.8)
project(movement)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
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

# action
add_library(action_coordinate SHARED
        src/coorServer.cpp)
target_include_directories(action_coordinate PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_coordinate
        PRIVATE "ACTION_COORDINATE_CPP_BUILDING_DLL")
ament_target_dependencies(action_coordinate
        "interfaces"
        "geometry_msgs"
        "turtlesim"
        "rclcpp"
        "rclcpp_action"
        "rclcpp_components")
rclcpp_components_register_node(action_coordinate PLUGIN "action_coordinate_cpp::CoordinateAction" EXECUTABLE action_coordinate_server)
install(TARGETS
        action_coordinate
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_package()
