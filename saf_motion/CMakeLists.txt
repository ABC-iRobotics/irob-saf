cmake_minimum_required(VERSION 3.16)
project(saf_motion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------------
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
find_package(saf_msgs REQUIRED)
find_package(saf_utils REQUIRED)
find_package(saf_robot REQUIRED)

include_directories(include)

# --------------------------------------------------------------------------
# surgeme_client library
# --------------------------------------------------------------------------
add_library(surgeme_client_lib
        src/surgeme_client.cpp
)
add_library(saf_motion::surgeme_client_lib ALIAS surgeme_client_lib)

target_include_directories(surgeme_client_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(surgeme_client_lib Eigen3::Eigen)
ament_target_dependencies(surgeme_client_lib
        rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
        tf2 tf2_ros tf2_geometry_msgs Eigen3
        saf_msgs saf_utils saf_robot
)
# --------------------------------------------------------------------------
# surgeme_server node
# --------------------------------------------------------------------------
add_executable(surgeme_server_node
        src/surgeme_server.cpp
)
target_link_libraries(surgeme_server_node
        surgeme_client_lib
        Eigen3::Eigen
)
ament_target_dependencies(surgeme_server_node
        rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
        tf2 tf2_ros tf2_geometry_msgs saf_msgs saf_utils saf_robot
)

# --------------------------------------------------------------------------
# mimic node
# --------------------------------------------------------------------------
add_executable(mimic_node
        src/mimic.cpp
)
target_link_libraries(mimic_node
        surgeme_client_lib
        Eigen3::Eigen
)
ament_target_dependencies(mimic_node
        rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
        tf2 tf2_ros tf2_geometry_msgs saf_msgs saf_utils saf_robot
)

# --------------------------------------------------------------------------
# Install & export
# --------------------------------------------------------------------------

# --- Install the library and export it ---
install(TARGETS
        surgeme_client_lib
        EXPORT saf_motionTargets           # ✅ consistent export set name
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# --- Install executables (not exported) ---
install(TARGETS
        surgeme_server_node
        mimic_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# --- Install headers & config files ---
install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

# --- Generate and export CMake config files ---
ament_export_targets(saf_motionTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
        rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
        tf2 tf2_ros tf2_geometry_msgs Eigen3
        saf_msgs saf_utils saf_robot
)

ament_package()