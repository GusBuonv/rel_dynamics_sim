cmake_minimum_required(VERSION 3.0)
project(rel_dynamics_sim)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  nearlab_msgs
  std_msgs
  geometry_msgs
  roscpp
  nearlab_utils
)

find_library(NL_UTILS NAMES nearlab_utils)

#find_package(Eigen3 3.3 REQUIRED NO_MODULE)
set(EIGEN3_INCLUDE_DIRS /usr/share/eigen)

catkin_package(CATKIN_DEPENDS
  std_msgs
  nearlab_msgs
  geometry_msgs
  roscpp
  message_runtime
  nearlab_utils
)

#########
# BUILD #
#########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(rel_dynamics_node
               src/relDynamicsNode.cpp)

set(EXEC_LIST rel_dynamics_node)
foreach(EXEC_FILE ${EXEC_LIST})
  target_link_libraries(${EXEC_FILE} ${catkin_LIBRARIES} ${NL_UTILS})
  #add_dependencies(${EXEC_FILE} ${catkin_iEXPORTED_TARGETS})
  install(TARGETS ${EXEC_FILE}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(EXEC_FILE ${EXEC_LIST})
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   PATTERN ".svn" EXCLUDE
# )
