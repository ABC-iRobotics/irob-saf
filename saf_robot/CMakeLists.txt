cmake_minimum_required(VERSION 3.16)
project(saf_robot)

# --------------------------------------------------------------------
# Compiler setup
# --------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(saf_msgs REQUIRED)
find_package(saf_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# --------------------------------------------------------------------
# Library: saf_dvrk_lib
# --------------------------------------------------------------------
add_library(saf_dvrk_lib
        src/saf_dvrk/arm_types.cpp
)

target_include_directories(saf_dvrk_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(saf_dvrk_lib
        rclcpp geometry_msgs sensor_msgs Eigen3
)

target_link_libraries(saf_dvrk_lib Eigen3::Eigen)

# --------------------------------------------------------------------
# Executable: robot_server_dvrk
# --------------------------------------------------------------------
add_executable(robot_server_dvrk
        src/saf_dvrk/robot_server_psm.cpp
)

ament_target_dependencies(robot_server_dvrk
        rclcpp rclcpp_action
        std_msgs geometry_msgs sensor_msgs
        action_msgs saf_msgs saf_utils Eigen3
)

target_link_libraries(robot_server_dvrk
        saf_dvrk_lib
        Eigen3::Eigen
)

# --------------------------------------------------------------------
# Install & Export
# --------------------------------------------------------------------
install(TARGETS
        saf_dvrk_lib
        EXPORT export_saf_dvrk_lib        # <---- Export target name
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
        robot_server_dvrk
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

# Proper export blocks (needed by dependent packages)
ament_export_targets(export_saf_dvrk_lib HAS_LIBRARY_TARGET)
ament_export_libraries(saf_dvrk_lib)
ament_export_dependencies(
        rclcpp rclcpp_action std_msgs geometry_msgs
        sensor_msgs action_msgs saf_msgs saf_utils Eigen3
)

ament_package()
