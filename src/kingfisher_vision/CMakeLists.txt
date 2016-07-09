cmake_minimum_required(VERSION 2.8.3)
project(kingfisher_vision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 ## pointgrey_camera_driver
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  image_transport
  cv_bridge
  OpenCV
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kingfisher
#  CATKIN_DEPENDS pointgrey_camera_driver roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ executable

add_executable(image_converter src/image_converter.cpp)
target_link_libraries(image_converter ${catkin_LIBRARIES})

add_executable(kingfisher_vision_long src/kingfisher_vision_long.cpp)
target_link_libraries(kingfisher_vision_long ${catkin_LIBRARIES})

add_executable(kingfisher_vision_short src/kingfisher_vision_short.cpp)
target_link_libraries(kingfisher_vision_short ${catkin_LIBRARIES})

add_executable(position_filter_short src/position_filter_short.cpp)
target_link_libraries(position_filter_short ${catkin_LIBRARIES})

add_executable(position_filter_long src/position_filter_long.cpp)
target_link_libraries(position_filter_long ${catkin_LIBRARIES})


add_executable(sort_test src/sort_test.cpp)
target_link_libraries(sort_test ${catkin_LIBRARIES})


#add_executable(ocam_functions src/ocam_functions.cpp)
#target_link_libraries(ocam_functions ${catkin_LIBRARIES})

