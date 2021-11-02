project(chassis_ros)
cmake_minimum_required(VERSION 3.10.0)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wall -g -std=c++11 -O0 ")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  roslib
  nav_msgs
  tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
catkin_package(LIBRARIES ${PROJECT_NAME})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)


aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src DIR_SRCS)
add_executable(${PROJECT_NAME}_node ${DIR_SRCS} main.cpp)
target_link_libraries(${PROJECT_NAME}_node m chassis ${catkin_LIBRARIES})

add_custom_command(
	OUTPUT copy_lib
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/lib/* ${CATKIN_DEVEL_PREFIX}/lib
)
add_custom_target(COPY_CHASSIS_LIB ALL DEPENDS copy_lib ${PROJECT_NAME}_node)