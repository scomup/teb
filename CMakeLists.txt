cmake_minimum_required(VERSION 2.8.12) 

project(teb_demo)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O0")
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3")

# Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5Widgets REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Ceres REQUIRED COMPONENTS SparseLinearAlgebraLibrary)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)


add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs  message_filters laser_geometry visualization_msgs  rosbag)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_BINARY_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${LUA_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})

file(GLOB_RECURSE ALL_SRCS "*.cc")



add_executable(${PROJECT_NAME}  ${ALL_SRCS})

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME}
${LUA_LIBRARIES}
${catkin_LIBRARIES}
${Boost_LIBRARIES}
${PCL_LIBRARIES}
${CERES_LIBRARIES}
${Pangolin_LIBRARIES}
Qt5::Widgets
pthread
dl
)

