cmake_minimum_required(VERSION 2.8.3)
project(iop_component_fkie)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  iop_builder_fkie
  pluginlib
  roscpp
)

iop_init_paths()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS pluginlib roscpp
)


include_directories(${catkin_INCLUDE_DIRS}
                    ${IOP_COMMON_INCLUDES}
                    include
)

add_executable(iop_component
               src/iop_component.cpp
               src/main.cpp
)
# Specify libraries to link a library or executable target against
target_link_libraries(iop_component
                      ${IOP_LIBRARIES}
                      ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}
            src/iop_component.cpp
)

target_link_libraries(${PROJECT_NAME}
                      ${IOP_LIBRARIES}
                      ${catkin_LIBRARIES}
)


# Mark headers for installation
install(
  DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

# Mark executables and/or libraries for installation
install(TARGETS iop_component
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

