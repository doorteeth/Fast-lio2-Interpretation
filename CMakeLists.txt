cmake_minimum_required(VERSION 3.10)
project(fast_lio)

SET(CMAKE_BUILD_TYPE "Debug")

add_compile_options(-std=c++14)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O0 -pthread -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES  "(x86)|(X86)|(amd64)|(AMD64)" )
    include(ProcessorCount)
    ProcessorCount(N)
    message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
)
find_package(Eigen3 REQUIRED)
find_package(PCL  REQUIRED)

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include)

add_message_files(
    FILES
    Pose6D.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

file(GLOB_RECURSE ALL_CPP_FILES src/*.cpp)
message(STATUS "找到的 C++ 文件: ${ALL_CPP_FILES}")
add_library(slam ${ALL_CPP_FILES})

add_executable(fastlio_maapping app/laserMapping.cpp)
target_link_libraries(fastlio_maapping slam
    ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)
