cmake_minimum_required(VERSION 3.0.2)
project(carrot_team)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  trajectory_msgs
  message_generation
  std_msgs
)

add_message_files(
    FILES
    poi.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs#   trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES go_to_poi
#  CATKIN_DEPENDS geometry_msgs roscpp rospy trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vehicle_to_poi
        src/vehicle_to_poi.cpp
)
target_link_libraries(vehicle_to_poi
        ${catkin_LIBRARIES}
)
add_executable(keyboard_control
	src/keyboard_control.cpp
	)
target_link_libraries(keyboard_control
	${catkin_LIBRARIES}
	)
