cmake_minimum_required(VERSION 3.8)
project(irob_robot)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(irob_msgs REQUIRED)
find_package(irob_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# Testing
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
# Libraries
# ------------------------------------------------------------------------------

# irob_dvrk_lib
set(IROB_DVRK_SOURCES
        src/irob_dvrk/robot_server_dvrk.cpp
        src/irob_dvrk/arm_types.cpp
)

add_library(irob_dvrk_lib SHARED ${IROB_DVRK_SOURCES})

ament_target_dependencies(irob_dvrk_lib
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
        action_msgs
        irob_msgs
        irob_utils
)

target_include_directories(irob_dvrk_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(irob_dvrk_lib Eigen3::Eigen)

# irob_general_robot_lib
set(IROB_GENERAL_ROBOT_SOURCES
        src/irob_general_robot/robot_client.cpp
)

add_library(irob_general_robot_lib SHARED ${IROB_GENERAL_ROBOT_SOURCES})

ament_target_dependencies(irob_general_robot_lib
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
        action_msgs
        irob_msgs
        irob_utils
)

target_include_directories(irob_general_robot_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(irob_general_robot_lib Eigen3::Eigen)

# ------------------------------------------------------------------------------
# Executable
# ------------------------------------------------------------------------------

add_executable(robot_server_dvrk src/irob_dvrk/robot_server_psm.cpp)

ament_target_dependencies(robot_server_dvrk
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
        action_msgs
        irob_msgs
        irob_utils
)

target_link_libraries(robot_server_dvrk
        irob_dvrk_lib
        irob_general_robot_lib
        Eigen3::Eigen
)

# ------------------------------------------------------------------------------
# Installation
# ------------------------------------------------------------------------------

install(TARGETS
        irob_dvrk_lib
        irob_general_robot_lib
        robot_server_dvrk
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
)

# ------------------------------------------------------------------------------
# Export
# ------------------------------------------------------------------------------

ament_export_libraries(irob_dvrk_lib irob_general_robot_lib)
ament_export_dependencies(
        rclcpp std_msgs geometry_msgs sensor_msgs action_msgs irob_msgs irob_utils Eigen3
)
ament_package()
