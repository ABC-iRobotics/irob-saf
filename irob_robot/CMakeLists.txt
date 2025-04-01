cmake_minimum_required(VERSION 3.8)
project(irob_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(irob_msgs REQUIRED)
find_package(irob_utils REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include directories
include_directories(include)

# Define libraries
set(IROB_DVRK_SOURCES
        src/irob_dvrk/robot_server_psm.cpp
        src/irob_dvrk/robot_server_dvrk.cpp
        src/irob_dvrk/arm_types.cpp
)

add_library(irob_dvrk_lib ${IROB_DVRK_SOURCES})

set_target_properties(irob_dvrk_lib PROPERTIES LINKER_LANGUAGE CXX)

set(IROB_GENERAL_ROBOT_SOURCES
        src/irob_general_robot/robot_client.cpp
)

add_library(irob_general_robot_lib ${IROB_GENERAL_ROBOT_SOURCES})

set_target_properties(irob_general_robot_lib PROPERTIES LINKER_LANGUAGE CXX)

# Define Executables
add_executable(robot_server_dvrk
        src/irob_dvrk/robot_server_psm.cpp
)

# Link libraries to executables
target_link_libraries(irob_dvrk_lib
        ${rclcpp_LIBRARIES}
        ${rclpy_LIBRARIES}
)

target_link_libraries(irob_general_robot_lib
        ${rclcpp_LIBRARIES}
)

target_link_libraries(robot_server_dvrk
        ${rclcpp_LIBRARIES}
        irob_dvrk_lib
)

# Install libraries, executables, and headers
install(TARGETS irob_dvrk_lib irob_general_robot_lib robot_server_dvrk
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/irob_dvrk include/irob_general_robot
        DESTINATION include
)

# Install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.launch"
)

# Install configuration files
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/config
        FILES_MATCHING
        PATTERN "*.yaml"
        PATTERN "*.xml"
)


ament_package()
