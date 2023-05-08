cmake_minimum_required(VERSION 2.8.12) 

project(teb_demo)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -std=c++14 ")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -std=c++14")


find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTSAM REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs  message_filters laser_geometry visualization_msgs  rosbag)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_BINARY_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})
include_directories(${GTSAM_INCLUDE_DIR})

file(GLOB_RECURSE ALL_SRCS "*.cc")



add_executable(${PROJECT_NAME}  ${ALL_SRCS})

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
${Boost_LIBRARIES}
${YAML_CPP_LIBRARIES}
gtsam
)

