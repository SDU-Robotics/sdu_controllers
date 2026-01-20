cmake_minimum_required(VERSION 3.0.2)
project(test_cbf)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sdu_math
)
find_package(osqp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CGAL)
generate_messages(
  DEPENDENCIES std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES belt_task
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

#set(MOSEK_LIBRARY_DIRS "/home/yik/mosek/10.0/tools/platform/linux64x86/bin/")
#set(MOSEK_LIBRARIES
#"/home/yik/mosek/10.0/tools/platform/linux64x86/bin/libmosek64.so.10.0"
#)

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(/home/yik/osqp-cpp/include)
include_directories(
  include
  include/test_cbf
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
set( SOURCE_FILES 
 src/test_cbf.cpp
 src/control_barrier_functions.cpp
)

#link_directories(/home/yik/mosek/10.0/tools/platform/linux64x86/bin/)
add_executable(test_cbf ${SOURCE_FILES})
#target_link_libraries(test_cbf ${catkin_LIBRARIES} rtde osqp::osqp mosek64 fusion64 cgal CGAL::CGAL)
target_link_libraries(test_cbf ${catkin_LIBRARIES} rtde quadprog)
#add_dependencies(test_mosek mosek64)
