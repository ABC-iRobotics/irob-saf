cmake_minimum_required(VERSION 3.8)
project(irob_robot)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(irob_msgs REQUIRED)
find_package(irob_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# Test dependencies
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# irob_dvrk_lib sources and headers
set(IROB_DVRK_SOURCES
        src/irob_dvrk/robot_server_psm.cpp
        src/irob_dvrk/robot_server_dvrk.cpp
        src/irob_dvrk/arm_types.cpp
)

set(IROB_DVRK_HEADERS
        include/irob_dvrk/arm_types.hpp
        include/irob_dvrk/robot_server_dvrk.hpp
        include/irob_dvrk/robot_server_psm.hpp
)

# Create library
add_library(irob_dvrk_lib SHARED ${IROB_DVRK_SOURCES} ${IROB_DVRK_HEADERS})

ament_target_dependencies(irob_dvrk_lib rclcpp irob_utils irob_msgs)

# irob_general_robot_lib sources and headers
set(IROB_GENERAL_ROBOT_SOURCES
        src/irob_general_robot/robot_client.cpp
)

set(IROB_GENERAL_ROBOT_HEADERS
        include/irob_general_robot/robot_client.hpp
        include/irob_general_robot/robot_server.hpp
)

# Create library
add_library(irob_general_robot_lib SHARED ${IROB_GENERAL_ROBOT_SOURCES} ${IROB_GENERAL_ROBOT_HEADERS})

ament_target_dependencies(irob_general_robot_lib rclcpp irob_utils irob_msgs)

# Define Executables
add_executable(robot_server_dvrk src/irob_dvrk/robot_server_psm.cpp)

# Link libraries to executables
target_link_libraries(robot_server_dvrk
        irob_dvrk_lib
        irob_general_robot_lib
)

# Export the libraries
ament_export_libraries(irob_dvrk_lib irob_general_robot_lib)

# Install targets
install(TARGETS
        irob_dvrk_lib
        irob_general_robot_lib
        robot_server_dvrk
        DESTINATION lib
)

# Install header files
install(DIRECTORY include/irob_dvrk include/irob_general_robot
        DESTINATION include
)

# Install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
