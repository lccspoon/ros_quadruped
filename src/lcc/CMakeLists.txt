# 设置CMake最低版本要求
cmake_minimum_required(VERSION 3.0.2)

# 定义项目名称
project(lcc)

# 设置C++编译选项：优化级别为-O3，启用所有警告，使用C++11标准
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")
add_compile_options(-std=c++11)

# 查找所需的catkin包（ROS相关）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  gazebo_msgs
  geometry_msgs
  sensor_msgs
)

# 查找依赖库：Eigen3、OsqpEigen、Boost
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# 配置Pinocchio库
find_package(pinocchio QUIET)
if(NOT pinocchio_FOUND)
  # 如果未找到Pinocchio的CMake配置文件，手动指定路径
  set(PINOCCHIO_INCLUDE_DIRS /opt/pinocchio/include)
  set(PINOCCHIO_LIBRARY_DIRS /usr/local/lib)
  set(PINOCCHIO_LIBRARIES -lpinocchio_default -lpinocchio_parsers)  # 只保留必要的库
else()
  message(STATUS "找到Pinocchio: ${pinocchio_VERSION}")
  set(PINOCCHIO_INCLUDE_DIRS ${pinocchio_INCLUDE_DIRS})
  set(PINOCCHIO_LIBRARY_DIRS ${pinocchio_LIBRARY_DIRS})
  set(PINOCCHIO_LIBRARIES ${pinocchio_LIBRARIES})
endif()

# 声明catkin包
catkin_package()

# 设置头文件包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /usr/local/include/osqp
)

# 设置库文件链接目录
link_directories(
  ${PINOCCHIO_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

# 搜索所有源文件
file(GLOB_RECURSE SRC_LIST "src/*/*.cpp" "src/*/*.cc")

# 生成可执行文件
add_executable(main src/main.cpp ${SRC_LIST})

# 链接所需库
target_link_libraries(main
  ${catkin_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
  ${Boost_LIBRARIES}
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)