cmake_minimum_required(VERSION 2.8.3)
project(iop_transport_fkie)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS iop_component_fkie roscpp)
iop_init(COMPONENT_ID 0)

iop_export_service(urn_jaus_jss_core_Transport)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS iop_component_fkie
)

###########
## Build ##
###########
iop_code_generator(
  IDLS
    urn.jaus.jss.core-v1.1/Transport.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_core_Transport/TransportService.h
    src/main.cpp
  EXTERN_SERVICES
  GENERATED_SOURCES cppfiles
)

include_directories(${catkin_INCLUDE_DIRS} ${EXPORT_INCLUDE_DIR})

add_library(${PROJECT_NAME}
            src/TransportPlugin.cpp
            ${cppfiles}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
)

install(
  DIRECTORY ${IOP_INSTALL_INCLUDE_DIRS} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
   FILES ./plugin_iop.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
