cmake_minimum_required(VERSION 3.4)
project(oakd_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(depthai CONFIG REQUIRED)
if(depthai_FOUND)
  message(WARNING "depthai FOUND, Version:" ${depthai_VERSION} " directory: " ${depthai_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
  message_generation
  message_runtime
)

find_package(OpenCV REQUIRED)
if(OPENCV_FOUND)
  message(WARNING "OpenCV FOUND, Version:" ${OpenCV_VERSION} " directory: " ${OpenCV_INCLUDE_DIRS})
endif()



add_message_files(
  FILES
    bbox.msg
    bboxes.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main_ros.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS} depthai::core depthai::opencv)