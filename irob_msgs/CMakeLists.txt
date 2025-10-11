cmake_minimum_required(VERSION 3.8)
project(irob_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Only message dependencies here (no need for rclcpp in a pure interface package)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ------------------------------------------------------------------------------
# Interface generation
# ------------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Point2D.msg"
        "msg/ToolPose.msg"
        "msg/ToolPoseStamped.msg"
        "msg/TrajectoryToolPose.msg"
        "msg/VisionObject.msg"
        "msg/InstrumentJawPart.msg"
        "msg/InstrumentInfo.msg"
        "msg/FloatArray.msg"
        "msg/GraspObject.msg"
        "msg/Environment.msg"
        "msg/Marker.msg"
        "msg/MarkerArray.msg"

        "srv/GetControlVariables.srv"
        "srv/BoolQuery.srv"

        "action/Robot.action"
        "action/Surgeme.action"

        DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

# ------------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
# Export for other packages
# ------------------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()
