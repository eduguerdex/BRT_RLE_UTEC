cmake_minimum_required(VERSION 3.0.2)
project(BRT_Bringup)

find_package(catkin REQUIRED COMPONENTS
 rospy roscpp sensor_msgs  std_msgs
)
## Declare a catkin package
catkin_package()

install(DIRECTORY config launch meshes src urdf publish
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(publish publish/publish.cpp)
target_link_libraries(publish ${catkin_LIBRARIES})
add_dependencies(publish std_msgs)
