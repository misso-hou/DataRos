cmake_minimum_required(VERSION 3.12)
project(offline_ros)

# 设置 C++ 标准 //NOTE:ros noetic默认使用c++14
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)
# 可见性编译报警消除
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

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
find_package(Protobuf REQUIRED)

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

# 添加要构建的proto源代码文件
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
# 编译proto文件
file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
# 把生成的头文件路径加入包含目录
include_directories(${CMAKE_CURRENT_BINARY_DIR})
# 定义公共源文件（包含proto生成的源文件）
set(COMMON_SRC_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/function/ros_topic_parser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/function/animation.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/function/animation_functions.cpp
  ${PROTO_SRCS}  # 添加proto生成的源文件
)

# --------------------转向数据播放器节点--------------------
add_executable(steer_play_node
  ${COMMON_SRC_FILES}
  src/RosNode/steer_play_node.cpp       
  src/function/display_control.cpp
)

# ROS 与外部库的链接
target_link_libraries(steer_play_node
  ${COMMON_LIBS}
  ${VISUALIZATION_LIBS}
)

# --------------------刹车数据播放器节点--------------------
add_executable(brake_play_node
  ${COMMON_SRC_FILES}
  src/RosNode/brake_play_node.cpp      
  src/function/display_control.cpp
  src/algorithm/observer.cpp
)

# ROS 与外部库的链接
target_link_libraries(brake_play_node
  ${COMMON_LIBS}
  ${VISUALIZATION_LIBS}
)

# --------------------转向实时运算节点--------------------
add_executable(steer_realtime_node
  ${COMMON_SRC_FILES}
  src/RosNode/steer_realtime_node.cpp      
)

# ROS 与外部库的链接
target_link_libraries(steer_realtime_node
  ${COMMON_LIBS}
  ${VISUALIZATION_LIBS}
)

# --------------------刹车实时运算节点--------------------
add_executable(brake_realtime_node
  ${COMMON_SRC_FILES}
  src/RosNode/brake_realtime_node.cpp      
  src/algorithm/observer.cpp
)

# ROS 与外部库的链接
target_link_libraries(brake_realtime_node
  ${COMMON_LIBS}
  ${VISUALIZATION_LIBS}
)

# --------------------HMI实时运算节点--------------------
add_executable(hmi_realtime_node
  ${COMMON_SRC_FILES}
  src/RosNode/hmi_realtime_node.cpp      
)

# ROS 与外部库的链接
target_link_libraries(hmi_realtime_node
  ${COMMON_LIBS}
  ${VISUALIZATION_LIBS}
)


# # 测试节点
# add_executable(proto_test 
#   src/RosNode/proto_test.cpp 
#   ${PROTO_SRCS} ${PROTO_HDRS}
# )
# target_link_libraries(proto_test ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

###########################INSTALL###########################
# 安装执行文件
install(TARGETS 
  steer_play_node 
  brake_play_node 
  steer_realtime_node 
  brake_realtime_node
  hmi_realtime_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

