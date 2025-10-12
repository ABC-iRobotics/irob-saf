cmake_minimum_required(VERSION 3.8)
project(irob_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(irob_msgs REQUIRED)
find_package(irob_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# --------------------------------------------------------------------
# Library: arm types
# --------------------------------------------------------------------
add_library(irob_dvrk_lib
        src/irob_dvrk/arm_types.cpp
)
ament_target_dependencies(irob_dvrk_lib
        rclcpp geometry_msgs sensor_msgs Eigen3
)
target_include_directories(irob_dvrk_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(irob_dvrk_lib Eigen3::Eigen)

# --------------------------------------------------------------------
# Executable: robot_server_psm
# --------------------------------------------------------------------
add_executable(robot_server_psm src/irob_dvrk/robot_server_psm.cpp)
ament_target_dependencies(robot_server_psm
        rclcpp rclcpp_action
        std_msgs geometry_msgs sensor_msgs
        action_msgs irob_msgs irob_utils Eigen3
)
target_link_libraries(robot_server_psm irob_dvrk_lib Eigen3::Eigen)

# --------------------------------------------------------------------
# Install & export
# --------------------------------------------------------------------
install(TARGETS
        irob_dvrk_lib
        robot_server_psm
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(irob_dvrk_lib)
ament_export_dependencies(rclcpp rclcpp_action std_msgs geometry_msgs
        sensor_msgs action_msgs irob_msgs irob_utils Eigen3)
ament_package()
