cmake_minimum_required(VERSION 2.8.3)
project(irob_subtask_logic)
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
	sensor_msgs
	cv_bridge
	actionlib
	actionlib_msgs
	irob_utils
	irob_motion
	irob_vision_support
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




## Generate added messages and services with any dependencies listed here
#generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)

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
	INCLUDE_DIRS include
	LIBRARIES irob_behavior_lib
	CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs sensor_msgs
	message_runtime actionlib_msgs irob_utils
	#DEPENDS system_lib
	)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} 
	/usr/include/eigen3
	include
	)

## Declare a C++ library

file(GLOB irob_subtask_logic_lib_src
	"src/*.cpp"
	)


add_library( irob_subtask_logic_lib STATIC
	${irob_subtask_logic_lib_src}
	)

set_target_properties( irob_subtask_logic_lib  PROPERTIES LINKER_LANGUAGE CXX)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(irob_dvrk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Declare a C++ executable
#add_executable(move_test src/move_test.cpp)
#add_executable(move_circles src/irob_autosurg_execute/move_circles.cpp)
#add_executable(log_trajectory src/log_trajectory.cpp)
#add_executable(rviz_plot_trajectory src/rviz_plot_trajectory.cpp)
#add_executable(play_logged_trajectory src/play_logged_trajectory.cpp)

add_executable(camera src/camera.cpp)
add_executable(peg_transfer_unilateral src/peg_transfer_unilateral.cpp)
add_executable(peg_transfer_bilateral src/peg_transfer_bilateral.cpp)
#add_executable(peg_transfer_dual src/peg_transfer_dual.cpp)
#add_executable(cut_vessel src/cut_vessel.cpp)
add_executable(grasp src/grasp.cpp)
add_executable(accuracy_meas src/accuracy_meas.cpp)


#add_executable(auto_dissection src/auto_dissection.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(irob_dvrk_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




## Specify libraries to link a library or executable target against


target_link_libraries(irob_subtask_logic_lib
	${catkin_LIBRARIES}
	)

target_link_libraries(peg_transfer_unilateral
        ${catkin_LIBRARIES}
        irob_subtask_logic_lib
        )

target_link_libraries(peg_transfer_bilateral
	${catkin_LIBRARIES}
	irob_subtask_logic_lib
	)

target_link_libraries(camera
	${catkin_LIBRARIES}
	irob_subtask_logic_lib
	)
 # target_link_libraries(peg_transfer_dual
 #     ${catkin_LIBRARIES}
 #     irob_subtask_logic_lib
 #     )

  #  target_link_libraries(cut_vessel
 #       ${catkin_LIBRARIES}
  #      irob_subtask_logic_lib
  #      )


target_link_libraries(grasp
	${catkin_LIBRARIES}
	irob_subtask_logic_lib
	)


  target_link_libraries(accuracy_meas
      ${catkin_LIBRARIES}
      irob_subtask_logic_lib
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
