cmake_minimum_required(VERSION 2.8.3)
project(nao_testbed)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  nao_utilities
)

set(CMAKE_BUILD_TYPE Debug)

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

#---------------------------Touch and talk module-----------------------------#
add_executable(nao_testbed_touch_and_talk
  src/touch_and_talk/main.cpp
  src/touch_and_talk/module.cpp
)
add_dependencies(nao_testbed_touch_and_talk 
  nao_msgs_gencpp # wait for nao_msgs to be build
  nao_utilities_gencpp # wait for nao_utilities to be build
) 
target_link_libraries(nao_testbed_touch_and_talk 
  ${catkin_LIBRARIES}
  nao_utilities_lib 
)
#----------------------------------Track face----------------------------------#
add_executable(nao_testbed_track_face
  src/track_face/main.cpp
  src/track_face/module.cpp
)
add_dependencies(nao_testbed_track_face 
  nao_msgs_gencpp # wait for nao_msgs to be build
  nao_utilities_gencpp # wait for nao_utilities to be build
) 
target_link_libraries(nao_testbed_track_face 
  ${catkin_LIBRARIES}
  nao_utilities_lib 
)
#---------------------Change posture by vocal commands------------------------#
add_executable(nao_testbed_vocal_commands_change_posture
  src/vocal_commands_posture_change/main.cpp
  src/vocal_commands_posture_change/module.cpp
)
add_dependencies(nao_testbed_vocal_commands_change_posture 
  nao_msgs_gencpp # wait for nao_msgs to be build
  nao_utilities_gencpp # wait for nao_utilities to be build
) 
target_link_libraries(nao_testbed_vocal_commands_change_posture 
  ${catkin_LIBRARIES}
  nao_utilities_lib 
)
