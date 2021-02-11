cmake_minimum_required(VERSION 3.5)
project(fkie_iop_builder)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-ignored-qualifiers)
endif()

find_package(ament_cmake REQUIRED)
find_package(jaustoolset REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include
                    ${rclcpp_INCLUDE_DIRS}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY cmake DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY jsidl DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(jaustoolset rclcpp)

ament_package(
    CONFIG_EXTRAS cmake/iop_build_macros.cmake.in
)
