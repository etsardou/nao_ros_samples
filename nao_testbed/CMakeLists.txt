cmake_minimum_required(VERSION 2.8.3)
project(nao_testbed)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  nao_utilities
)

set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    roscpp
    actionlib
    nao_utilities
)

#-----------------------------UTILITIES LIB---------------------------------#
add_library(nao_testbed_node
  src/main.cpp
)

add_dependencies(nao_testbed_node 
  nao_msgs_gencpp # wait for nao_msgs to be build
  nao_utilities_gencpp # wait for nao_utilities to be build
) 

target_link_libraries(nao_testbed_node 
  ${catkin_LIBRARIES} 
)
