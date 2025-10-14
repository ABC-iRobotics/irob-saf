cmake_minimum_required(VERSION 3.16)
project(irob_vision_support)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(irob_utils REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

# --------------------------------------------------------------------------
# Executables
# --------------------------------------------------------------------------
add_executable(camera_preprocessor_node src/camera_preprocessor.cpp)
ament_target_dependencies(camera_preprocessor_node
        rclcpp sensor_msgs irob_utils
)

add_executable(dummy_target_publisher_node src/dummy_target_publisher.cpp)
ament_target_dependencies(dummy_target_publisher_node
        rclcpp geometry_msgs visualization_msgs tf2 irob_utils Eigen3
)
target_link_libraries(dummy_target_publisher_node Eigen3::Eigen)
target_include_directories(dummy_target_publisher_node PUBLIC ${EIGEN3_INCLUDE_DIRS})

add_executable(dummy_image_processor_node src/dummy_image_processor.cpp)
ament_target_dependencies(dummy_image_processor_node
        rclcpp geometry_msgs visualization_msgs tf2
)

add_executable(serial_test_node src/serial_test.cpp)
ament_target_dependencies(serial_test_node
        rclcpp irob_utils
)

# --------------------------------------------------------------------------
# Install
# --------------------------------------------------------------------------
install(TARGETS
        camera_preprocessor_node
        dummy_target_publisher_node
        dummy_image_processor_node
        serial_test_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
        rclcpp sensor_msgs irob_utils geometry_msgs visualization_msgs
        tf2 tf2_geometry_msgs Eigen3
)

ament_package()
