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
  message_generation
)

# generate messages

add_message_files(
  FILES
  Person.msg
  PersonCoordinate.msg
  RegionOfInterestInfo.msg
  RegionOfInterestInfos.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

find_package(PCL 1.8)
if(NOT PCL_FOUND)
  message(WARNING "PCL 1.8 not found, disabled")
  find_package(PCL 1.7)
  if(NOT PCL_FOUND)
    message(WARNING "PCL not found, disabled")
  else()
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
  endif()
else()
  set(PCL_1.8_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif()

find_package(OpenCV 3)
if(NOT OpenCV_FOUND)
  message(WARNING "opencv 3 not found, disabled")
  find_package(OpenCV)
  if(NOT OpenCV_FOUND)
    message(WARNING "opencv not found, disabled")
  else()
    include_directories(${OpenCV_INCLUDE_DIRS})
  endif()
else()
  set(OpenCV_3_FOUND)
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

if (linux_kinect_FOUND AND OpenCV_3_FOUND AND PCL_1.8_FOUND)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES roboenvcv roboenvcv_extra
 CATKIN_DEPENDS
 geometry_msgs roscpp roslib sensor_msgs std_msgs tf
)

################## :optional ##################

elseif (OpenCV_3_FOUND AND PCL_1.8_FOUND)
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
if (linux_kinect_FOUND AND OpenCV_3_FOUND AND PCL_1.8_FOUND)
  add_library(roboenvcv_extra
    src/roboenvcv_extra.cc
  )
endif()
################## :optional ##################

if (OpenCV_3_FOUND AND PCL_1.8_FOUND)
  add_library(roboenvcv
    src/roboenvcv.cc
  )
endif()

################## nodes: ##################
find_package(darknet_ros_msgs)

if (darknet_ros_msgs_FOUND AND PCL_FOUND)
  add_executable(darknet_faceinfo_extractor
    nodes/darknet_faceinfo_extractor.cc)
  target_link_libraries(darknet_faceinfo_extractor
    ${catkin_LIBRARIES} ${PCL_LIBRARIES})
endif()

if (OpenCV_FOUND AND PCL_FOUND)
  add_executable(haar_faceinfo_extractor
    nodes/haar_faceinfo_extractor.cc)
  target_link_libraries(haar_faceinfo_extractor
    ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
endif()

add_executable(crop_image nodes/crop_image.cc)
target_link_libraries(crop_image ${catkin_LIBRARIES})

add_executable(static_sensor_tf nodes/static_sensor_tf.cc)
target_link_libraries(static_sensor_tf ${catkin_LIBRARIES})

add_executable(to_global nodes/to_global.cc)
target_link_libraries(to_global ${catkin_LIBRARIES})

add_executable(id_mapper nodes/id_mapper.cc)
target_link_libraries(id_mapper ${catkin_LIBRARIES})
################## nodes: ##################
