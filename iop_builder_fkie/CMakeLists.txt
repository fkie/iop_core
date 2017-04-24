cmake_minimum_required(VERSION 2.8.3)
project(iop_builder_fkie)

find_package(catkin REQUIRED)
catkin_package(
    INCLUDE_DIRS include
    CFG_EXTRAS iop_build_macros.cmake
)

install(
  DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
    PROGRAMS
        cmake/chmerge.py
        cmake/cpp.py
        cmake/cpy.py
        cmake/mss.py
        scripts/jaus_node_manager.sh
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
)

install(
  DIRECTORY jsidl DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
