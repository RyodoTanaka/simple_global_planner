cmake_minimum_required(VERSION 2.8.3)
project(simple_global_planner)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  costmap_2d
  nav_core
  geometry_msgs
  tf2
  tf2_ros
  base_local_planner
  pluginlib
  )

catkin_package(
  LIBRARIES simple_global_planner
  CATKIN_DEPENDS
    roscpp
    costmap_2d
    nav_core
    geometry_msgs
    tf2
    tf2_ros
    base_local_planner
    pluginlib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(simple_global_planner src/simple_global_planner.cpp)
target_link_libraries(simple_global_planner ${catkin_LIBRARIES})


#############
## Install ##
#############
install(TARGETS simple_global_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
