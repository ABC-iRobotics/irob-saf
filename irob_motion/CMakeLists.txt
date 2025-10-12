cmake_minimum_required(VERSION 3.16)
project(irob_motion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(irob_msgs REQUIRED)
find_package(irob_utils REQUIRED)
find_package(irob_robot REQUIRED)

include_directories(include)

# --- surgeme_client as a library (like ROS1 "used by others") ---
add_library(surgeme_client_lib
        src/surgeme_client.cpp
)
target_link_libraries(surgeme_client_lib Eigen3::Eigen)
ament_target_dependencies(surgeme_client_lib
        rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
        tf2 tf2_ros tf2_geometry_msgs
        irob_msgs irob_utils irob_robot
)

# --- surgeme_server node ---
add_executable(surgeme_server_node
        src/surgeme_server.cpp
)
target_link_libraries(surgeme_server_node
        surgeme_client_lib
        Eigen3::Eigen
)
ament_target_dependencies(surgeme_server_node
        rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
        tf2 tf2_ros tf2_geometry_msgs
        irob_msgs irob_utils irob_robot
)

# --- mimic node ---
add_executable(mimic_node
        src/mimic.cpp
)
target_link_libraries(mimic_node
        surgeme_client_lib
        Eigen3::Eigen
)
ament_target_dependencies(mimic_node
        rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
        tf2 tf2_ros tf2_geometry_msgs
        irob_msgs irob_utils irob_robot
)

# Install
install(TARGETS
        surgeme_client_lib
        surgeme_server_node
        mimic_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
        rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
        tf2 tf2_ros tf2_geometry_msgs Eigen3 irob_msgs irob_utils irob_robot
)

ament_package()
