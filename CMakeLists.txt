cmake_minimum_required(VERSION 2.8.3)
project(poscmd_2_odom)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  #pid_planner
  base_local_planner
  geometry_msgs
  tf2
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

#add_executable (poscmd_2_odom src/poscmd_2_odom.cpp)
#target_link_libraries(poscmd_2_odom
#  ${catkin_LIBRARIES})

#add_executable(follow_point src/follow_point.cpp)
#target_link_libraries(follow_point ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(pid_planner src/pid_planner.cpp)
target_link_libraries(pid_planner ${catkin_LIBRARIES})

#add_executable(turtlebot_test src/turtlebot_test.cpp)
#target_link_libraries(turtlebot_test ${catkin_LIBRARIES})
