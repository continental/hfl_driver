cmake_minimum_required(VERSION 3.5.1)

project(hfl_driver)

## Compile as C++11, supported in ROS kinetic  and newer
add_compile_options(-std=c++11)

## Subdirectories
add_subdirectory(hfl_utilities)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslint
  tf
  dynamic_reconfigure
  nodelet
  roscpp
  rospy
  std_msgs
  message_generation
  image_transport
  camera_info_manager
  cv_bridge
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS)

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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
  DIRECTORY msg
  FILES 
  Status.msg
  ToFImage.msg
  PixelData.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/HFL.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hfl_utilities
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  visualization_msgs
  angles
  tf2
  tf2_geometry_msgs
  arpa
  image_transport
  camera_info_manager
  message_generation
  udp_com
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} 
  src/camera_commander/camera_commander.cpp
  src/image_processor/hfl110dcu.cpp
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable()

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} 
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  hfl_utilities
  udp_com
)

#############
## Testing ##
#############

# ROS linter
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  # Add rostest gtest
  add_rostest_gtest(tests_hfl110dcu
    test/hfl110dcu.test 
    test/hfl110dcu-test.cpp
    test/hfl110dcu-utils-test.cpp
    test/test_main.cpp)

  target_link_libraries(tests_hfl110dcu
	  ${PROJECT_NAME}
	  ${catkin_LIBRARIES}
	  hfl_utilities)
endif()

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
 install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
   #DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   # FILES_MATCHING PATTERN "*.h"
   # PATTERN ".svn" EXCLUDE

## Mark other files for installation (e.g. launch and bag files, etc.)

#Launch Files 
install(FILES
  launch/hfl110dcu.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
 )
#nodelets.xml
install(FILES
  nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#configuration files
install(FILES
  cfg/camera_commander.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg/
)

#extrinsics parameters
install(FILES
  params/extrinsics/left.yaml
  params/extrinsics/right.yaml
  params/extrinsics/rear.yaml
  params/extrinsics/front.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params/extrinsics
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

####################
## Deb Generation ##
####################

set(CPACK_GENERATOR "DEB")

#Creating UNIX structured package
set(CPACK_CMAKE_GENERATOR "Unix Makefiles")
set(CPACK_PACKAGE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/dist")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})

#Set directory to /../ to avoid /share part of ROS_PACKAGE_PATH
set(CPACK_PACKAGING_INSTALL_PREFIX "$ENV{ROS_PACKAGE_PATH}/../")
set(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};${PROJECT_NAME};ALL;/./")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "${PROJECT_DESCRIPTION}")
set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_VENDOR "RCS")

#Now we can simply create the dependencies using almost the same package names as they declared in ROS repository
set(CPACK_DEBIAN_PACKAGE_DEPENDS "ros-melodic-roscpp, ros-melodic-std-msgs, ros-melodic-message-runtime, ros-melodic-sensor-msgs")

#IMPORTANT: set this to keep on auto-introspection for system dependencies like libc++6
set (CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

#Keep Debian package structure untouched
set(CPACK_DEBIAN_PACKAGE_CONTROL_STRICT_PERMISSION TRUE)

#Standard naming schema for Debian-based distros
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}")
set(CPACK_PACKAGE_VENDOR "Continental LiDAR Segment")
set(CPACK_PACKAGE_CONTACT "<moises.diaz@continental-corporation.com>")

#Turn the process on    
include(CPack)
