cmake_minimum_required(VERSION 3.0.2)
project(limo_follower)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  message_generation
  dynamic_reconfigure
)

# 生成消息
add_message_files(
  FILES
  position.msg
)

# 生成动态参数配置
generate_dynamic_reconfigure_options(
  config/laser_params.cfg
)

# 生成依赖
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs std_msgs message_runtime dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 安装脚本
catkin_install_python(PROGRAMS
  scripts/laser_tracker.py
  scripts/laser_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
