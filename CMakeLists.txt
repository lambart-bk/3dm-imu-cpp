cmake_minimum_required(VERSION 2.8.3)
project(microstrain_3dmgx2_imu)


#find_package()

find_library(LOG4CXX_LIBRARY log4cxx)
if(NOT LOG4CXX_LIBRARY)
  message(FATAL_ERROR "Couldn't find log4cxx library")
endif()


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include )

## Declare a cpp library
add_library(3dmgx2 src/3dmgx2.cc)

## Declare a cpp executable
add_executable(imu_node src/imu_node.cc)
target_link_libraries(imu_node 3dmgx2 )
#add_dependencies(imu_node microstrain_3dmgx2_imu_gencpp)

# Utility to recover ID from IMU
#add_executable(get_id src/get_id.cpp)
#target_link_libraries(get_id 3dmgx2 ${catkin_LIBRARIES} ${LOG4CXX_LIBRARY})




