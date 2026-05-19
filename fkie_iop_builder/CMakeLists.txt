cmake_minimum_required(VERSION 3.5)
project(fkie_iop_builder)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-ignored-qualifiers)
endif()

find_package(ament_cmake REQUIRED)
find_package(jaustoolset REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} INTERFACE jaustoolset::jaustoolset rclcpp::rclcpp)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY jsidl
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  DESTINATION lib
)
install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_export_include_directories(include ${jaustoolset_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS})
ament_export_dependencies(jaustoolset rclcpp)

ament_package(
    CONFIG_EXTRAS cmake/iop_build_macros.cmake.in
)
