cmake_minimum_required(VERSION 2.8.3)
project(roboenvcv)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
  cmake_modules
)

find_package(PCL 1.8)
if(NOT PCL_FOUND)
  message(WARNING "PCL not found, disabled")
else()
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif()

find_package(OpenCV 3)
if(NOT OpenCV_FOUND)
  message(WARNING "opencv not found, disabled")
else()
  include_directories(${OpenCV_INCLUDE_DIRS})
  message(STATUS "OpenCV library status:")
  message(STATUS "    version: ${OpenCV_VERSION}")
  message(STATUS "    libraries: ${OpenCV_LIBS}")
  message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
endif()

################## optional: ##################

find_package(linux_kinect)

if(linux_kinect_FOUND)
  include_directories(${linux_kinect_INCLUDE_DIRS})
  link_directories(${linux_kinect_LIBRARIES})
endif()

if (linux_kinect_FOUND AND OpenCV_FOUND AND PCL_FOUND)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES roboenvcv roboenvcv_extra
 CATKIN_DEPENDS
 geometry_msgs roscpp roslib sensor_msgs std_msgs tf
)

################## :optional ##################

elseif (OpenCV_FOUND AND PCL_FOUND)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES roboenvcv
 CATKIN_DEPENDS
 geometry_msgs roscpp roslib sensor_msgs std_msgs tf
)
else ()
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
 geometry_msgs roscpp roslib sensor_msgs std_msgs tf
)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

################## optional: ##################
if (linux_kinect_FOUND AND OpenCV_FOUND AND PCL_FOUND)
  add_library(roboenvcv_extra
    src/roboenvcv_extra.cc
  )
endif()
################## :optional ##################

if (OpenCV_FOUND AND PCL_FOUND)
  add_library(roboenvcv
    src/roboenvcv.cc
  )
endif()
