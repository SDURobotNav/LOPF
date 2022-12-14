cmake_minimum_required(VERSION 2.8.3)
project(hdl_people_tracking)

# -mavx causes a lot of errors!!
add_definitions(-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(OpenCV 3.4.10 REQUIRED)

find_package(Qt5Gui)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Test)
find_package(Qt5Concurrent)
find_package(Qt5OpenGL)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_ros
  roscpp
  sensor_msgs
  message_generation
  visualization_msgs
  geometry_msgs
  
)

########################
## message generation ##
########################
add_message_files(FILES
  Cluster.msg
  ClusterArray.msg
  Track.msg
  TrackArray.msg
  myClusters.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)


###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES hdl_people_tracking
   CATKIN_DEPENDS nodelet pcl_ros roscpp sensor_msgs message_runtime
#   DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(hdl_people_detection_nodelet
  apps/hdl_people_detection_nodelet.cpp
  src/kidono_human_classifier.cpp
  src/people_detector.cpp
  
)
target_link_libraries(hdl_people_detection_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(hdl_people_detection_nodelet hdl_people_tracking_gencpp)

add_library(hdl_people_tracking_nodelet apps/hdl_people_tracking_nodelet.cpp)
target_link_libraries(hdl_people_tracking_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(hdl_people_tracking_nodelet hdl_people_tracking_gencpp)
add_executable(mini_control
  src/mini_control.cpp
  src/input.cc
)
target_link_libraries(mini_control
  ${catkin_LIBRARIES}
)

add_executable(pos2vel
  src/pos2vel.cpp
)
target_link_libraries(pos2vel
  ${catkin_LIBRARIES}
)