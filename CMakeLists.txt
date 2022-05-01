cmake_minimum_required(VERSION 3.0.2)
project(pcl_voxel_reduce)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  tf
  tf_conversions
)
catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_voxel src/pcl_voxel.cpp)
target_link_libraries(pcl_voxel ${catkin_LIBRARIES})
add_dependencies(pcl_voxel ${catkin_EXPORTED_TARGETS})


install(TARGETS pcl_voxel
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
