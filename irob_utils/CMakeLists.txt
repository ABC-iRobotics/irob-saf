cmake_minimum_required(VERSION 3.8)
project(irob_utils)

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
find_package(irob_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Test dependencies
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Source files
file(GLOB irob_utils_lib_src "src/*.cpp")

# Library
add_library(irob_utils_lib SHARED ${irob_utils_lib_src})
ament_target_dependencies(irob_utils_lib
        rclcpp
        geometry_msgs
        std_msgs
        sensor_msgs
        irob_msgs
)

target_link_libraries(irob_utils_lib Eigen3::Eigen)

target_include_directories(irob_utils_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)

# Install library
install(TARGETS irob_utils_lib
        DESTINATION lib
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.hpp"
)

# Install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.launch.py"
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs geometry_msgs sensor_msgs irob_msgs Eigen3)
ament_export_libraries(irob_utils_lib)
ament_package()
