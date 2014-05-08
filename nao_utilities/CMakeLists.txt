cmake_minimum_required(VERSION 2.8.3)
project(nao_utilities)
find_package(catkin REQUIRED)

find_package(catkin 
  REQUIRED COMPONENTS
    roscpp
    actionlib
    nao_msgs
)

set(CMAKE_BUILD_TYPE Release)

#include directories for this package
include_directories(  
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    actionlib
    nao_msgs
    
  #export of directories for other packages in ws
  INCLUDE_DIRS  
    include
)

#-----------------------------UTILITIES LIB---------------------------------#
add_library(nao_utilities_lib 
  src/nao_speech.cpp
)

add_dependencies(nao_utilities_lib 
  nao_msgs_gencpp # wait for nao_msgs to be build
) 

target_link_libraries(nao_utilities_lib 
  ${catkin_LIBRARIES}
)
