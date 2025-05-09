cmake_minimum_required(VERSION 3.8)
project(alfred_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

add_library(${PROJECT_NAME} SHARED 
  src/alfred_interface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE 
  include
  ${SERIAL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${SERIAL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface alfred_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
