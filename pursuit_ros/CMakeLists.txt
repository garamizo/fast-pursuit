cmake_minimum_required(VERSION 2.8.3)
project(pursuit_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  pursuit_msgs
)

find_package(PCL 1.3 REQUIRED COMPONENTS common io)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pursuit
 CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs pursuit_msgs
)

###########
## Build ##
###########

include_directories(
  # include
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/../pursuit/include/GamePhysics
  ${PROJECT_SOURCE_DIR}/../pursuit/include/Pursuit
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(pursuit_lib
  ../pursuit/include/GamePhysics/Geometry3D.cpp
  ../pursuit/include/GamePhysics/matrices.cpp
  ../pursuit/include/GamePhysics/vectors.cpp
  ../pursuit/include/Pursuit/Map.cpp
  ../pursuit/include/Pursuit/Pursuit.cpp
)

add_library(pursuit_wrapper_lib
  src/PursuitWrapper.cpp
)
target_link_libraries(pursuit_wrapper_lib
  ${catkin_LIBRARIES}
)

add_executable(pursuit_ros src/pursuit_ros.cpp)
add_dependencies(pursuit_ros
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pursuit_ros
  ${catkin_LIBRARIES}
  pursuit_lib
  pursuit_wrapper_lib
)

add_executable(plot_rviz src/plot_rviz.cpp)
add_dependencies(plot_rviz
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(plot_rviz
  ${catkin_LIBRARIES}
  pursuit_lib
  pursuit_wrapper_lib
)


