cmake_minimum_required(VERSION 3.0.2)
project(irob_vision_support)
set(CMAKE_CXX_FLAGS -std=c++14)
#set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	COMPONENTS
	roscpp
	rospy
	roslib
        dynamic_reconfigure
	geometry_msgs
	std_msgs
	sensor_msgs
	stereo_msgs
	sensor_msgs
	cv_bridge
	image_transport
	camera_info_manager
	irob_msgs
	irob_utils
        pcl_ros
        #libpointmatcher_ros
	)


find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(realsense2 REQUIRED)
find_package(libpointmatcher 1.1.0 REQUIRED)

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
generate_dynamic_reconfigure_options(
   cfg/Fiducials.cfg
)

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
	LIBRARIES irob_vision_support_lib
	CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs sensor_msgs
        message_runtime irob_msgs irob_utils pcl_ros
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

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

include_directories("${libpointmatcher_INCLUDE_DIRS}")

## Declare a C++ library

file(GLOB irob_vision_support_lib_src
	"src/*.cpp"
	)


add_library( irob_vision_support_lib STATIC
	${irob_vision_support_lib_src}
	)

set_target_properties( irob_vision_support_lib  PROPERTIES LINKER_LANGUAGE CXX)


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


add_executable(vision_server_test_dummy
    src/vision_server_test_dummy.cpp)

add_executable(dummy_target_publisher
    src/dummy_target_publisher.cpp)

add_executable(camera_preprocessor
    src/camera_preprocessor.cpp)

add_executable(serial_test
    src/serial_test.cpp)

#add_executable(peg_transfer_perception
#    src/peg_transfer_perception.cpp)

#add_executable(auto_dissection src/auto_dissection.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(fiducial_detector.py ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




## Specify libraries to link a library or executable target against


target_link_libraries(irob_vision_support_lib
	${catkin_LIBRARIES}
	)


target_link_libraries(vision_server_test_dummy
	${catkin_LIBRARIES}
	irob_vision_support_lib
	)

target_link_libraries(dummy_target_publisher
	${catkin_LIBRARIES}
	irob_vision_support_lib
	)


target_link_libraries(camera_preprocessor
	${catkin_LIBRARIES}
	irob_vision_support_lib
        )



target_link_libraries(serial_test
        ${catkin_LIBRARIES}
        irob_vision_support_lib
        )

#target_link_libraries(peg_transfer_perception
#        ${catkin_LIBRARIES}
#        ${PCL_LIBRARIES}
#        ${realsense2_LIBRARY}
#        ${libpointmatcher_LIBRARIES}
#        irob_vision_support_lib
 #       )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 catkin_install_python(PROGRAMS
   scripts/fiducial_detector.py
   scripts/record_realsense_bag.py
   scripts/hand_eye_registrator.py
   scripts/block_detector.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark executables and/or libraries for installation
 install(PROGRAMS
    scripts/gst_endoscope.sh
   #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(PROGRAMS
##    scripts/gst_endoscope.sh
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
