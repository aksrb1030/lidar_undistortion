cmake_minimum_required(VERSION 2.8.3)
project(undistortion)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -o3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  laser_geometry
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  image_transport
  rospy
  std_msgs
  
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(OpenMP)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()



include_directories(
  "/usr/include/eigen3"
  src
  include
  /usr/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

# add_executable(undistortion
#   src/mt_filter.cpp
#   src/mt_mapbuilder.cpp
#   src/mt_lss.cpp
#   src/lss_node.cpp
# )



catkin_package(
  
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs cv_bridge 
  DEPENDS PCL OpenCV 
  INCLUDE_DIRS include
  LIBRARIES undistortion

)

add_executable(undistortion
  src/undistortion.cpp
)
target_link_libraries(undistortion
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  /usr/lib/liblas.so
)

install(TARGETS undistortion
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
 