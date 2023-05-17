cmake_minimum_required(VERSION 3.5)
project(cpp_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

set(text_publisher_executable string)
set(text_publisher_library ${text_publisher_executable}_core)
set(dependencies
  rclcpp
  rclcpp_components
  std_msgs)
  
include_directories(include)

add_executable(${text_publisher_executable}
  src/text_publisher_node.cpp
  src/text_publisher_component.cpp)
add_library(${text_publisher_library} SHARED
  src/text_publisher_component.cpp)
ament_target_dependencies(${text_publisher_executable}
  ${dependencies})
ament_target_dependencies(${text_publisher_library}
  ${dependencies})
rclcpp_components_register_nodes(${text_publisher_library}
  "cpp_test::TextPublisher")
  
install(TARGETS ${text_publisher_executable}
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${text_publisher_library}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include/)


ament_export_include_directories(include)
ament_export_libraries(${text_publisher_library})
ament_export_dependencies(${dependencies})
ament_package()
