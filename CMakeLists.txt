cmake_minimum_required(VERSION 2.8.3)
project(agv)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  message_generation
  dynamic_reconfigure
  autoware_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  EncoderCount.msg
  Waypoint.msg
  WaypointArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


######################################################## Build Nodes ##################################################################

add_executable(joystick src/joystick.cpp)
target_link_libraries(joystick ${catkin_LIBRARIES})
add_dependencies(joystick ${agv_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

add_executable(arduino_encoder_timer src/arduino_encoder_timer.cpp)
target_link_libraries(arduino_encoder_timer ${catkin_LIBRARIES})
add_dependencies(arduino_encoder_timer agv_generate_messages_cpp)

add_executable(encoder_odometry_system src/encoder_odometry_system.cpp)
target_link_libraries(encoder_odometry_system ${catkin_LIBRARIES})
add_dependencies(encoder_odometry_system agv_generate_messages_cpp)

add_executable(camera_projector src/camera_projector.cpp)
target_link_libraries(camera_projector ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(camera_projector agv_generate_messages_cpp)