cmake_minimum_required(VERSION 3.5.1)

project(hfl_driver)

add_compile_options(-std=c++11)

add_subdirectory(hfl_utilities)

find_package(catkin REQUIRED COMPONENTS
  tf
  tf2
  tf2_geometry_msgs 
  dynamic_reconfigure
  nodelet
  roscpp
  rospy
  std_msgs
  image_transport
  image_geometry
  camera_info_manager
  cv_bridge
  udp_com
)

generate_dynamic_reconfigure_options(
  cfg/HFL.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hfl_utilities
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  visualization_msgs
  angles
  tf
  tf2
  tf2_geometry_msgs
  image_transport
  image_geometry
  camera_info_manager
  udp_com
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
  src/camera_commander/camera_commander.cpp
  src/image_processor/hfl110dcu.cpp
)

add_dependencies(${PROJECT_NAME} 
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  hfl_utilities
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)
  # ROS linter
  roslint_cpp()
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

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/hfl110dcu.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
 )

install(FILES
  nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
