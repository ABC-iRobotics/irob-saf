cmake_minimum_required(VERSION 2.8.3)
project(irob_autosurg)
set(CMAKE_CXX_FLAGS -std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
                        COMPONENTS
                        roscpp
                        rospy
                        roslib
                        geometry_msgs
                        std_msgs
                        sensor_msgs
                        stereo_msgs
                        message_generation
                        actionlib_msgs
                        actionlib
                        dvrk_kinematics
                        sensor_msgs
						cv_bridge
						image_transport
						camera_info_manager
                        )


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ToolPose.msg
   ToolPoseStamped.msg
   TrajectoryToolPose.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   TargetPose.srv
   BoolQuery.srv
 )

## Generate actions in the 'action' folder
 add_action_files(
 	DIRECTORY
 	action
   	FILES
   	
   	robot/FollowTrajectory.action
   	robot/InitArm.action
   	robot/ResetPose.action
   	
   	gesture/Gesture.action
   	
   	maneuver/Grasp.action
   	maneuver/Dissect.action
   	maneuver/MoveTo.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  		#INCLUDE_DIRS include "${CATKIN_DEVEL_PREFIX}/include"
  		#LIBRARIES 
        CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs sensor_msgs
        	 message_runtime actionlib_msgs
        #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} 
								include/Eigen 
								include/irob_dvrk
								include/irob_utils
								include/irob_motion
								)

## Declare a C++ library

file(GLOB irob_vision_lib_src
    "src/irob_vision/*.cpp"
)


add_library( irob_vision_lib STATIC
       ${irob_vision_lib_src}
       )

set_target_properties( irob_vision_lib  PROPERTIES LINKER_LANGUAGE CXX)

file(GLOB irob_utils_lib_src
    "src/irob_utils/*.cpp"
)


add_library( irob_utils_lib STATIC
       ${irob_utils_lib_src}
       )

set_target_properties( irob_utils_lib  PROPERTIES LINKER_LANGUAGE CXX)


file(GLOB irob_dvrk_lib_src
    "src/irob_dvrk/*.cpp"
)


add_library( irob_dvrk_lib STATIC
       ${irob_dvrk_lib_src}
       )

set_target_properties( irob_dvrk_lib  PROPERTIES LINKER_LANGUAGE CXX)

file(GLOB irob_motion_lib_src
    "src/irob_motion/*.cpp"
)


add_library( irob_motion_lib STATIC
       ${irob_motion_lib_src}
       )

set_target_properties( irob_motion_lib  PROPERTIES LINKER_LANGUAGE CXX)

file(GLOB irob_task_lib_src
    "src/irob_task/*.cpp"
)


add_library( irob_task_lib STATIC
       ${irob_task_lib_src}
       )

set_target_properties( irob_task_lib  PROPERTIES LINKER_LANGUAGE CXX)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(irob_dvrk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(irob_dvrk_lib irob_autosurg_generate_messages_cpp)
add_dependencies(irob_utils_lib irob_autosurg_generate_messages_cpp)
add_dependencies(irob_motion_lib irob_autosurg_generate_messages_cpp)
add_dependencies(irob_task_lib irob_autosurg_generate_messages_cpp)
add_dependencies(irob_vision_lib irob_autosurg_generate_messages_cpp)

## Declare a C++ executable
#add_executable(move_test src/move_test.cpp)
#add_executable(move_circles src/irob_autosurg_execute/move_circles.cpp)
#add_executable(log_trajectory src/log_trajectory.cpp)
#add_executable(rviz_plot_trajectory src/rviz_plot_trajectory.cpp)
#add_executable(play_logged_trajectory src/play_logged_trajectory.cpp)


add_executable(dvrk_interface src/irob_dvrk/psm.cpp)

add_executable(move_circles_action_client src/irob_autosurg_execute/move_circles_action_client.cpp)
add_executable(gesture_server src/irob_motion/gesture_server.cpp)
add_executable(maneuver_server src/irob_motion/maneuver_server.cpp)


add_executable(test_gesture_client src/irob_motion/test_gesture_client.cpp)

add_executable(test_maneuver src/irob_motion/test_maneuver.cpp)

add_executable(test_task src/irob_task/test_task.cpp)

add_executable(retractor src/irob_task/retractor.cpp)

add_executable(vision_server_test_dummy 
				src/irob_vision/vision_server_test_dummy.cpp)
				
add_executable(dummy_target_publisher 
				src/irob_vision/dummy_target_publisher.cpp)
				
add_executable(camera_rotator 
				src/irob_vision/camera_rotator.cpp)
				
add_executable(stereo_sync
				src/irob_vision/stereo_sync.cpp)

#add_executable(auto_dissection src/auto_dissection.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(irob_dvrk_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(test_gesture_client irob_autosurg_generate_messages_cpp)
add_dependencies(dvrk_interface irob_autosurg_generate_messages_cpp)
add_dependencies(gesture_server irob_autosurg_generate_messages_cpp)
add_dependencies(maneuver_server irob_autosurg_generate_messages_cpp)

add_dependencies(test_task irob_autosurg_generate_messages_cpp)
add_dependencies(retractor irob_autosurg_generate_messages_cpp)
add_dependencies(vision_server_test_dummy irob_autosurg_generate_messages_cpp)


## Specify libraries to link a library or executable target against


 target_link_libraries(irob_vision_lib
 	${catkin_LIBRARIES}
 )
 
 target_link_libraries(irob_utils_lib
 	${catkin_LIBRARIES}
 )

 target_link_libraries(irob_dvrk_lib
 	${catkin_LIBRARIES}
 	irob_utils_lib
 )
 
 target_link_libraries(irob_motion_lib
 	${catkin_LIBRARIES}
 	irob_utils_lib
 )
 
 target_link_libraries(dvrk_interface
   ${catkin_LIBRARIES}
    irob_dvrk_lib
 )
 
  target_link_libraries(irob_task_lib
 	${catkin_LIBRARIES}
 	irob_utils_lib
 	irob_vision_lib
 	irob_motion_lib
 )
 
 
 target_link_libraries(move_circles_action_client
   ${catkin_LIBRARIES}
    irob_dvrk_lib
 )
 
 target_link_libraries(gesture_server
   ${catkin_LIBRARIES}
   irob_utils_lib
   irob_motion_lib
 )
 
 target_link_libraries(maneuver_server
   ${catkin_LIBRARIES}
   irob_utils_lib
   irob_motion_lib
 )
 
 target_link_libraries(test_gesture_client
   ${catkin_LIBRARIES}
   irob_utils_lib
   irob_motion_lib
 )
 
 target_link_libraries(test_maneuver
   ${catkin_LIBRARIES}
   irob_utils_lib
   irob_motion_lib
 )
 
 target_link_libraries(test_task
   ${catkin_LIBRARIES}
   irob_utils_lib
   irob_motion_lib
   irob_vision_lib
   irob_task_lib
 )
 
 target_link_libraries(retractor
   ${catkin_LIBRARIES}
   irob_utils_lib
   irob_motion_lib
   irob_vision_lib
   irob_task_lib
 )
 
 
 target_link_libraries(vision_server_test_dummy
   ${catkin_LIBRARIES}
   irob_utils_lib
   irob_vision_lib
 )
 
  target_link_libraries(dummy_target_publisher
   ${catkin_LIBRARIES}
   irob_utils_lib
   irob_vision_lib
 )
 
   target_link_libraries(camera_rotator
   ${catkin_LIBRARIES}
   irob_utils_lib
   irob_vision_lib
 )
 
    target_link_libraries(stereo_sync
   ${catkin_LIBRARIES}
   irob_utils_lib
   irob_vision_lib
 )
 
 

 


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS irob_dvrk irob_dvrk_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_irob_dvrk.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
