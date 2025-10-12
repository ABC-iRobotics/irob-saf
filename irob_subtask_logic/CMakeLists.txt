cmake_minimum_required(VERSION 3.16)
project(irob_subtask_logic)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# iRob core packages
find_package(irob_msgs REQUIRED)
find_package(irob_utils REQUIRED)
find_package(irob_motion REQUIRED)
find_package(irob_robot REQUIRED)
find_package(irob_vision_support REQUIRED)

include_directories(include)


# --------------------------------------------------------------------------
# Executables
# --------------------------------------------------------------------------

# --- Peg Transfer: Unilateral ---------------------------------------------
add_executable(peg_transfer_unilateral_node src/peg_transfer_unilateral.cpp)
ament_target_dependencies(peg_transfer_unilateral_node
        rclcpp rclcpp_action
        std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
        irob_msgs irob_utils irob_motion irob_robot irob_vision_support Eigen3
)
target_link_libraries(peg_transfer_unilateral_node
        irob_motion::surgeme_client_lib
        Eigen3::Eigen
)

# --- Peg Transfer: Bilateral ----------------------------------------------
add_executable(peg_transfer_bilateral_node src/peg_transfer_bilateral.cpp)
ament_target_dependencies(peg_transfer_bilateral_node
        rclcpp rclcpp_action
        std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
        irob_msgs irob_utils irob_motion irob_robot irob_vision_support Eigen3
)
target_link_libraries(peg_transfer_bilateral_node
        irob_motion::surgeme_client_lib
        Eigen3::Eigen
)

# --- Grasp ----------------------------------------------------------------
add_executable(grasp_node src/grasp.cpp)
ament_target_dependencies(grasp_node
        rclcpp rclcpp_action Eigen3
        irob_motion irob_utils irob_robot irob_vision_support
)
target_link_libraries(grasp_node
        irob_motion::surgeme_client_lib
        Eigen3::Eigen
)

# (Future executables like cut_vessel_node, camera_node, etc. can be added below.)

# --------------------------------------------------------------------------
# Installation
# --------------------------------------------------------------------------
install(TARGETS
        peg_transfer_unilateral_node
        peg_transfer_bilateral_node
        grasp_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------------------------------
# Exports
# --------------------------------------------------------------------------
ament_export_include_directories(include)
ament_export_dependencies(
        rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs stereo_msgs
        tf2 tf2_ros tf2_geometry_msgs Eigen3
        irob_msgs irob_utils irob_motion irob_robot irob_vision_support
)

ament_package()
