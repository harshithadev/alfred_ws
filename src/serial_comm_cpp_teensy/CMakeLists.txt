cmake_minimum_required(VERSION 3.8)
project(serial_comm_cpp_teensy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)


# Include libserial
find_package(PkgConfig REQUIRED)
pkg_check_modules(LibSerial REQUIRED libserial)

add_executable(serial_comm_cpp_teensy src/serial_comm_cpp_teensy.cpp)

# Link libraries
target_include_directories(serial_comm_cpp_teensy PRIVATE ${LibSerial_INCLUDE_DIRS})
target_link_libraries(serial_comm_cpp_teensy ${LibSerial_LIBRARIES})

ament_target_dependencies(serial_comm_cpp_teensy rclcpp geometry_msgs)

install(TARGETS
serial_comm_cpp_teensy
  DESTINATION lib/${PROJECT_NAME})

ament_package()

