cmake_minimum_required(VERSION 3.0.2)
project(map_loader)


find_package(GeographicLib REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS  
  nav_msgs  
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf  
  pcl_conversions
  pcl_ros
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
)

catkin_package(
  CATKIN_DEPENDS 
  nav_msgs 
  roscpp 
  sensor_msgs 
  geometry_msgs 
  std_msgs 
  tf  
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  LIBRARIES
)

include_directories(
  include
  maps
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(map_loader src/map_loader.cpp)
target_link_libraries(map_loader ${catkin_LIBRARIES} ${GeographicLib_LIBRARY_DIRS}/libGeographic.so)
add_dependencies(map_loader ${catkin_EXPORTED_TARGETS})



install(
  TARGETS
  map_loader    
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


