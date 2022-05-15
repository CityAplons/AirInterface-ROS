cmake_minimum_required(VERSION 2.8.3)
project(airinterface)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  tf
  message_generation 
  std_msgs 
  std_srvs 
  geometry_msgs 
  mavros_msgs 
)

catkin_python_setup()

add_service_files(
  FILES
  UnityGetState.srv
  UnitySetState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS src
CATKIN_DEPENDS message_runtime std_msgs mavros_msgs geometry_msgs)

FILE(GLOB BIN_FILES "bin/*")
install(PROGRAMS ${BIN_FILES} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})