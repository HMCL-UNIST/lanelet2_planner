cmake_minimum_required(VERSION 2.8.3)
project(autoware_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs  
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    DTLane.msg
    Lane.msg
    Waypoint.msg
    WaypointState.msg
)

# add_service_files(
#   FILES RecognizeLightState.srv
# )

generate_messages(
  DEPENDENCIES
    geometry_msgs    
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs    
    message_runtime
    sensor_msgs
    std_msgs
)
