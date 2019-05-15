cmake_minimum_required(VERSION 2.8.3)
project(publish_map)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
IF(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

find_package (catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
sensor_msgs
pcl_ros
cv_bridge
image_transport
octomap_msgs
nav_msgs
)

find_package(PCL 1.7 REQUIRED)

find_package(octomap REQUIRED)

catkin_package (
        CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport sensor_msgs pcl_ros octomap_msgs nav_msgs
)

include_directories(
${PCL_INCLUDE_DIRS}
${OCTOMAP_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)

set(LIBS_ROS
${PCL_LIBRARIES}
${OCTOMAP_LIBRARIES}
${catkin_LIBRARIES}
)
add_executable (${PROJECT_NAME}_exe  src/convert_map.cc)
target_link_libraries(${PROJECT_NAME}_exe ${LIBS_ROS})