cmake_minimum_required(VERSION 3.5)
project(fkie_iop_ocu_slavelib)

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
find_package(fkie_iop_discovery REQUIRED)
find_package(fkie_iop_management REQUIRED)
find_package(fkie_iop_msgs REQUIRED)

###############
## setup IOP ##
###############
iop_init()

###########
## Build ##
###########
set(dependencies
  fkie_iop_discovery
  fkie_iop_management
  fkie_iop_msgs
)

include_directories(
  include
  ${IOP_COMPONENT_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
            src/Component.cpp
            src/ServiceInfo.cpp
            src/Slave.cpp
            src/SlaveHandlerInterface.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

#############
## Install ##
#############

install(
  DIRECTORY ${IOP_INSTALL_INCLUDE_DIRS}
  DESTINATION include/${PROJECT_NAME}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include ${IOP_COMPONENT_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()