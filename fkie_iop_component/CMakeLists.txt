cmake_minimum_required(VERSION 3.5)
project(fkie_iop_component)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

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
find_package(diagnostic_msgs REQUIRED)
find_package(fkie_iop_builder REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(dependencies
    diagnostic_msgs
    fkie_iop_builder
    pluginlib
    rclcpp
)

iop_init_paths()

include_directories(
    ${diagnostic_msgs_INCLUDE_DIRS}
    ${fkie_iop_builder_INCLUDE_DIRS}
    ${jaustoolset_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    include
)


# create component executable
add_executable(iop_component
               src/iop_component.cpp
               src/iop_config.cpp
               src/main.cpp
               src/time.cpp
               src/timer.cpp
)
ament_target_dependencies(iop_component ${dependencies})

# create shared library
add_library(${PROJECT_NAME} SHARED
            src/iop_component.cpp
            src/iop_config.cpp
            src/time.cpp
            src/timer.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Mark headers for installation
install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
    PATTERN "*.old" EXCLUDE
    PATTERN "*.gen" EXCLUDE
)

install(
  FILES include/JConstants.h
  DESTINATION include/${PROJECT_NAME}
)

# Mark executables and/or libraries for installation
install(
    TARGETS iop_component ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
