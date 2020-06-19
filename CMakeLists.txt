cmake_minimum_required(VERSION 2.8.3)
project(remote_manipulation_markers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  interactive_markers
  message_generation
  pcl_conversions
  pcl_ros
  rail_grasp_calculation_msgs
  rail_manipulation_msgs
  roscpp
  sensor_msgs
  std_srvs
  tf
  tf_conversions
  tf2_bullet
  tf2_ros
  visualization_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  GenerateGrasps.msg
)

add_service_files(
  FILES
  CreateSphere.srv
  CycleGrasps.srv
  ModeSwitch.srv
)

add_action_files(
  FILES
  SpecifiedGrasp.action
  SpecifiedPose.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(free_positioning src/FreePositioning.cpp src/Common.cpp)
add_executable(constrained_positioning src/ConstrainedPositioning.cpp src/Common.cpp)
add_executable(gripper_marker_vis src/GripperMarkerVis.cpp src/Common.cpp)
add_executable(point_and_click src/PointAndClick.cpp src/PointAndClick.cpp src/Common.cpp)
add_executable(click_and_refine src/ClickAndRefine.cpp src/Common.cpp src/RefinablePose.cpp)
add_executable(test_click_and_refine src/TestClickAndRefine.cpp)
add_executable(clickable_point_cloud src/ClickablePointCloud.cpp src/point_cloud_manipulation.cpp)
add_executable(point_and_click_demo src/PointAndClickDemo.cpp)
add_executable(test_cloud src/TestCloud.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(free_positioning ${catkin_LIBRARIES})
target_link_libraries(constrained_positioning ${catkin_LIBRARIES})
target_link_libraries(gripper_marker_vis ${catkin_LIBRARIES})
target_link_libraries(point_and_click ${catkin_LIBRARIES})
target_link_libraries(click_and_refine ${catkin_LIBRARIES})
target_link_libraries(test_click_and_refine ${catkin_LIBRARIES})
target_link_libraries(clickable_point_cloud ${catkin_LIBRARIES})
target_link_libraries(point_and_click_demo ${catkin_LIBRARIES} ${EIGEN_INCLUDE_DIRS})
target_link_libraries(test_cloud ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
add_dependencies(free_positioning ${PROJECT_NAME}_generate_messages_cpp rail_manipulation_msgs_gencpp)
add_dependencies(constrained_positioning ${PROJECT_NAME}_generate_messages_cpp rail_manipulation_msgs_gencpp)
add_dependencies(point_and_click ${PROJECT_NAME}_generate_messages_cpp rail_manipulation_msgs_gencpp)
add_dependencies(click_and_refine ${PROJECT_NAME}_generate_messages_cpp rail_manipulation_msgs_gencpp)
add_dependencies(clickable_point_cloud ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(point_and_click_demo rail_manipulation_msgs_gencpp rail_grasp_calculation_msgs_gencpp
        ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS free_positioning constrained_positioning gripper_marker_vis point_and_click click_and_refine test_click_and_refine clickable_point_cloud
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
