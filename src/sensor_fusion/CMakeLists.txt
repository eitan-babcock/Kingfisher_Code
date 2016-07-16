cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cmake_modules
)

catkin_package(
#  INCLUDE_DIRS include 
  LIBRARIES sensor_fusion
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS 
  INCLUDE_DIRS ${Eigen_INCLUDE_DIRS}
)



# get eigen
find_package(Eigen REQUIRED)
include_directories(include ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


add_executable(oneDOF_dog_tracking src/oneDOF_dog_tracking.cpp)
target_link_libraries(oneDOF_dog_tracking ${catkin_LIBRARIES} )


add_executable(imu_vision_fusion src/imu_vision_fusion.cpp)
target_link_libraries(imu_vision_fusion ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS} )

add_executable(velocity_output src/velocity_output.cpp)
target_link_libraries(velocity_output ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS} )

add_executable(sensor_fusion src/sensor_fusion.cpp)
target_link_libraries(sensor_fusion ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS} )

add_executable(imu_vision_bias_fusion src/imu_vision_bias_fusion.cpp)
target_link_libraries(imu_vision_bias_fusion ${catkin_LIBRARIES} ${Eigen_INCLUDE_DIRS} )