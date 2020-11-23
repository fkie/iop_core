cmake_minimum_required(VERSION 3.0.2)
project(fkie_iop_builder)
find_package(catkin REQUIRED)

find_package(jaustoolset REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CFG_EXTRAS iop_build_macros.cmake
    CATKIN_DEPENDS jaustoolset
)

install(
  DIRECTORY
      include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY cmake DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY jsidl DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
