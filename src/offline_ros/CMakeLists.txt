cmake_minimum_required(VERSION 3.12)
project(offline_ros)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
)

# 查找外部库
find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(matplotlibcpp17 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# 定义公共依赖（多个节点共享）
set(COMMON_LIBS
  ${catkin_LIBRARIES}
  protobuf
  /home/workspace/common_protobuf/build/latest/libplusai_common_proto.so
)

# 定义可视化专用依赖
set(VISUALIZATION_LIBS
  ${Python3_LIBRARIES}
  matplotlibcpp17::matplotlibcpp17
  pybind11::embed
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

# 安装脚本或配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_node
  CATKIN_DEPENDS roscpp rospy std_msgs roslib
  # 其他 ROS 依赖
)

# 包含其他自定义的头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Python3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  /home/workspace/common_protobuf/build/latest
  /home/workspace/common_protobuf/build/latest/plusai_common_proto
)

# 添加要构建的源代码文件
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# add_executable(${PROJECT_NAME}_node ${SRC_FILES})
# --------------------转向数据播放器节点--------------------
add_executable(steer_play_node
  src/RosNode/steer_play_node.cpp      
  src/function/ros_topic_parser.cpp    
  src/function/animation.cpp
  src/function/display_control.cpp
)

# ROS 与外部库的链接
target_link_libraries(steer_play_node
  ${COMMON_LIBS}
  ${VISUALIZATION_LIBS}
)

# --------------------刹车数据播放器节点--------------------
add_executable(brake_play_node
  src/RosNode/brake_play_node.cpp      
  src/function/ros_topic_parser.cpp    
  src/function/animation.cpp
  src/function/display_control.cpp
  src/algorithm/observer.cpp
)

# ROS 与外部库的链接
target_link_libraries(brake_play_node
  ${COMMON_LIBS}
  ${VISUALIZATION_LIBS}
)

# --------------------实时运算节点--------------------
add_executable(realtime_node
  src/RosNode/realtime_node.cpp      
  src/function/ros_topic_parser.cpp    
  src/function/animation.cpp
)

# ROS 与外部库的链接
target_link_libraries(realtime_node
  ${COMMON_LIBS}
  ${VISUALIZATION_LIBS}
)

# --------------------ROS监听器节点-------------------
add_executable(ros_listener_node
  src/RosNode/ros_listener_node.cpp
  src/function/ros_topic_parser.cpp
)

target_link_libraries(ros_listener_node
  ${COMMON_LIBS}
)


###########################INSTALL###########################
# 安装执行文件
install(TARGETS steer_play_node brake_play_node ros_listener_node realtime_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

