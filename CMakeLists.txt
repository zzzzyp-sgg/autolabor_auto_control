cmake_minimum_required(VERSION 3.0)
project(autolabor_auto_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    motionState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp geometry_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(auto_control_node src/auto_control_node.cpp)

add_dependencies(auto_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(auto_control_node
  ${catkin_LIBRARIES}
)
