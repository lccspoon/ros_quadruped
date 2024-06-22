cmake_minimum_required(VERSION 3.0.2)
project(lcc)

# 设置编译类型为Release
if(NOT CMAKE_BUILD_TYPE)  
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")

# 使用C++11标准
add_compile_options(-std=c++11)

# 查找catkin包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  gazebo_msgs
  geometry_msgs
  sensor_msgs
)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

# 声明catkin包
catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES lcc
#   CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs gazebo_msgs geometry_msgs sensor_msgs
#   DEPENDS Eigen3 OsqpEigen
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
)

# 链接目录（如果需要的话）
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# 搜索源文件
file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

# 添加可执行文件
add_executable(main src/main.cpp ${SRC_LIST})

# 链接库
target_link_libraries(main
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)
