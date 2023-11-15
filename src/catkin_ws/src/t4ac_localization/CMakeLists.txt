cmake_minimum_required(VERSION 2.8.3)
project(t4ac_localization)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_generation
  #geographic_msgs
  #geodesy
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#)
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   #geographic_msgs
)

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
catkin_package(
	LIBRARIES ${PROJECT_NAME}
#        INCLUDE_DIRS include
        DEPENDS rt pthread
	DEPENDS message_runtime
	CATKIN_DEPENDS  roscpp
        #geographic_msgs
)

## Sources
#set(serial_SRCS
#    src/serial.cc
#    include/serial/serial.h
#    include/serial/v8stdint.h
#)
#list(APPEND serial_SRCS src/impl/unix.cc)
#list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)

## Add serial library
#add_library(${PROJECT_NAME} ${serial_SRCS})
#target_link_libraries(${PROJECT_NAME} rt pthread)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

##-------------  NODE1:  -------------------------------------
add_executable(t4ac_tf_broadcaster_node src/t4ac_tf_broadcaster.cpp)
target_link_libraries(t4ac_tf_broadcaster_node ${catkin_LIBRARIES})
##-------------  NODE2:  -------------------------------------
add_executable(t4ac_pose_rviz_publisher_node src/t4ac_pose_rviz_publisher.cpp)
target_link_libraries(t4ac_pose_rviz_publisher_node ${catkin_LIBRARIES})

## Include headers
include_directories(include)

## Install executable
#install(TARGETS ${PROJECT_NAME}
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

## Install headers
#install(FILES include/serial/serial.h include/serial/v8stdint.h
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/serial)

