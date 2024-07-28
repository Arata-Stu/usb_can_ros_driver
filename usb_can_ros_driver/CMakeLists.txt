cmake_minimum_required(VERSION 3.8)
project(usb_can_ros_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(usb_can_ros_msg REQUIRED)  # 追加

# CANlib
find_library(CANLIB_LIBRARY NAMES canlib libcanlib.so PATHS /usr/lib /usr/local/lib)

include_directories(
  include
  ${CANLIB_INCLUDE_DIRS}
)

#sender
add_executable(can_sender_node src/can_sender_node.cpp)

target_link_libraries(can_sender_node
  ${CANLIB_LIBRARY}
)
ament_target_dependencies(can_sender_node rclcpp usb_can_ros_msg)     # CHANGE

#receiver
add_executable(can_receiver_node src/can_receive_node.cpp)

target_link_libraries(can_receiver_node
  ${CANLIB_LIBRARY}
)
ament_target_dependencies(can_receiver_node rclcpp usb_can_ros_msg)     # CHANGE


install(TARGETS
  can_sender_node
  can_receiver_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
