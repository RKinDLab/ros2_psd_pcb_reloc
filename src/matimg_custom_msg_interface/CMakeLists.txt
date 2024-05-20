cmake_minimum_required(VERSION 3.8)
project(matimg_custom_msg_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Similar to https://github.com/ros2/common_interfaces/blob/fa1d0093db5df80c886093f27810d09c46f5111e/sensor_msgs/CMakeLists.txt#L67-L73
set(msg_files
  "msg/MatImg.msg" 
  "msg/EigenMsg.msg"
)

# https://github.com/NMBURobotics/ros2_python_demos/blob/main/ros2_demo_custom_msgs/CMakeLists.txt
rosidl_generate_interfaces(${PROJECT_NAME}
  # path to each custom message is added one per line
  ${msg_files}
  DEPENDENCIES 
  std_msgs 
  sensor_msgs
)

# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(my_node "${cpp_typesupport_target}") 
ament_export_dependencies(rosidl_default_runtime)
# Humble and newer uses the following: https://robotics.stackexchange.com/questions/23171/creating-a-custom-msg-in-ros2
ament_package()
