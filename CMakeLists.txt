cmake_minimum_required(VERSION 3.4)
project(oakd-ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14") # if want to use auto in Lamda function, should be more recent than c++14
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(depthai CONFIG REQUIRED)
if(depthai_FOUND)
  message(WARNING "depthai FOUND")
  message(WARNING ${depthai_VERSION})
  message(WARNING ${depthai_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(OpenCV4 REQUIRED)
if(OPENCV_FOUND)
  message(WARNING "OpenCV FOUND")
  message(WARNING ${OpenCV_VERSION})
  message(WARNING ${OpenCV_INCLUDE_DIRS})
endif()

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS} depthai::core depthai::opencv)
