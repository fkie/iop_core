cmake_minimum_required(VERSION 3.0.2)
project(fkie_iop_ocu_slavelib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
                    fkie_iop_accesscontrol
                    fkie_iop_discovery
                    fkie_iop_management
                    fkie_iop_msgs
                    roscpp)
find_package(Boost REQUIRED COMPONENTS system)

iop_init(COMPONENT_ID 0)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
          fkie_iop_accesscontrol
          fkie_iop_discovery
          fkie_iop_management
          fkie_iop_msgs
          roscpp
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
            src/Component.cpp
            src/ServiceInfo.cpp
            src/Slave.cpp
)

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

