cmake_minimum_required(VERSION 3.0.2)
project(motor_ros_serial)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
)




catkin_package(

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




# 添加可执行文件
add_executable(motor_serial src/motor_serial.cpp)

# 将serialport库链接到可执行文件
target_link_libraries(motor_serial ${catkin_LIBRARIES})
