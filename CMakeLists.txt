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
  UInt8ArrayStamped.msg
  Int32Stamped.msg
  BoolStamped.msg
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
  set(PCL_1.8_FOUND 1)
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
  set(OpenCV_3_FOUND 1)
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
 geometry_msgs roscpp roslib sensor_msgs std_msgs tf message_generation
)

################## :optional ##################

elseif (OpenCV_3_FOUND AND PCL_1.8_FOUND)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES roboenvcv
 CATKIN_DEPENDS
 geometry_msgs roscpp roslib sensor_msgs std_msgs tf message_generation
)
else ()
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
 geometry_msgs roscpp roslib sensor_msgs std_msgs tf message_generation
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
  add_dependencies(roboenvcv_extra roboenvcv_generate_messages_cpp)
endif()
################## :optional ##################

if (OpenCV_3_FOUND AND PCL_1.8_FOUND)
  add_library(roboenvcv
    src/roboenvcv.cc
  )
  add_dependencies(roboenvcv roboenvcv_generate_messages_cpp)
endif()

################## nodes: ##################
find_package(darknet_ros_msgs)

if (darknet_ros_msgs_FOUND AND PCL_FOUND)
  add_executable(darknet_faceinfo_extractor
    nodes/darknet_faceinfo_extractor.cc)
  target_link_libraries(darknet_faceinfo_extractor
    ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  add_dependencies(darknet_faceinfo_extractor roboenvcv_generate_messages_cpp)
endif()

if (OpenCV_FOUND AND PCL_FOUND)
  add_executable(haar_faceinfo_extractor
    nodes/haar_faceinfo_extractor.cc)
  target_link_libraries(haar_faceinfo_extractor
    ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
  add_dependencies(haar_faceinfo_extractor roboenvcv_generate_messages_cpp)
endif()

if (darknet_ros_msgs_FOUND AND OpenCV_FOUND AND PCL_FOUND)
  add_executable(darknethaar_faceinfo_extractor
    nodes/darknethaar_faceinfo_extractor.cc)
  target_link_libraries(darknethaar_faceinfo_extractor
    ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
  add_dependencies(darknethaar_faceinfo_extractor roboenvcv_generate_messages_cpp)
endif()

add_executable(crop_image nodes/crop_image.cc)
target_link_libraries(crop_image ${catkin_LIBRARIES})
add_dependencies(crop_image roboenvcv_generate_messages_cpp)

add_executable(static_sensor_tf nodes/static_sensor_tf.cc)
target_link_libraries(static_sensor_tf ${catkin_LIBRARIES})

add_executable(dynamic_sensor_tf nodes/dynamic_sensor_tf.cc)
target_link_libraries(dynamic_sensor_tf ${catkin_LIBRARIES})

add_executable(to_global nodes/to_global.cc)
target_link_libraries(to_global ${catkin_LIBRARIES})
add_dependencies(to_global roboenvcv_generate_messages_cpp)

add_executable(id_mapper nodes/id_mapper.cc)
target_link_libraries(id_mapper ${catkin_LIBRARIES})
add_dependencies(id_mapper roboenvcv_generate_messages_cpp)
################## nodes: ##################

################## nodes2: ##################
find_package(jsk_recognition_msgs)

if (jsk_recognition_msgs_FOUND AND PCL_FOUND AND OpenCV_FOUND)
  include_directories(${jsk_recognition_msgs_INCLUDE_DIRS})
  add_executable(openpose_faceinfo_extractor
    nodes2/openpose_faceinfo_extractor.cc)
  target_link_libraries(openpose_faceinfo_extractor
    ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
  add_dependencies(openpose_faceinfo_extractor roboenvcv_generate_messages_cpp)
endif()

add_executable(crop_image_timestamped nodes2/crop_image_timestamped.cc)
target_link_libraries(crop_image_timestamped ${catkin_LIBRARIES})
add_dependencies(crop_image_timestamped roboenvcv_generate_messages_cpp)

add_executable(dynamic_sensor_tfstamped nodes2/dynamic_sensor_tfstamped.cc)
target_link_libraries(dynamic_sensor_tfstamped ${catkin_LIBRARIES})

add_executable(to_global_dynamic nodes2/to_global_dynamic.cc)
target_link_libraries(to_global_dynamic ${catkin_LIBRARIES})
add_dependencies(to_global_dynamic roboenvcv_generate_messages_cpp)

add_executable(id_mapper2 nodes2/id_mapper2.cc)
target_link_libraries(id_mapper2 ${catkin_LIBRARIES})
add_dependencies(id_mapper2 roboenvcv_generate_messages_cpp)

add_executable(sensordirection_filter nodes2/sensordirection_filter.cc)
target_link_libraries(sensordirection_filter ${catkin_LIBRARIES})
add_dependencies(sensordirection_filter roboenvcv_generate_messages_cpp)

add_executable(sensordirection_axis_static nodes2/sensordirection_axis_static.cc)
target_link_libraries(sensordirection_axis_static ${catkin_LIBRARIES})
add_dependencies(sensordirection_axis_static roboenvcv_generate_messages_cpp)
################## :nodes2 ##################

################## else: ##################
if (PCL_1.8_FOUND)
  add_executable(euclidean_clustering else/euclidean_clustering.cc)
  target_link_libraries(euclidean_clustering ${catkin_LIBRARIES} ${PCL_LIBRARIES})
endif()
################## :else ##################
