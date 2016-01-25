cmake_minimum_required(VERSION 2.8.3)
project(senior_mapping)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
	message("OPENMP FOUND")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(CUDA REQUIRED)
if(CUDA_FOUND)
	message("CUDA FOUND")
	SET(CUDA_NVCC_FLAGS "-arch=sm_52" CACHE STRING "nvcc flags" FORCE)
	SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
	SET(LIB_TYPE STATIC) 
	CUDA_ADD_LIBRARY(TestLib ${LIB_TYPE} src/helloWorld.cu)
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES senior_mapping
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(pc_creator src/pc_creator.cpp)
target_link_libraries(pc_creator
  ${catkin_LIBRARIES}
)

add_executable(odom_converter src/odom_converter.cpp)
target_link_libraries(odom_converter
  ${catkin_LIBRARIES}
)

add_executable(map_creator src/map_creator.cpp)
target_link_libraries(map_creator
  ${catkin_LIBRARIES}
)



add_executable(main src/main.cpp)
add_dependencies(main TestLib)
target_link_libraries(main
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  TestLib
)
