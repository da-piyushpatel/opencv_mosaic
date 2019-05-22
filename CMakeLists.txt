cmake_minimum_required(VERSION 2.8.3)
project(realtimemosaic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  dynamic_reconfigure
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES realtimemosaic
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
) 
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
 )
add_executable(${PROJECT_NAME}_node src/realtimemosaic.cpp)
add_executable(mapper_node src/mapper.cpp)
add_executable(mapper_node2 src/mapper2.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

target_link_libraries(mapper_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

target_link_libraries(mapper_node2
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )