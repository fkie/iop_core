cmake_minimum_required(VERSION 3.5)
project(fkie_iop_discovery)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# set default c++ version
if($ENV{ROS_DISTRO} MATCHES "foxy")
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()
elseif($ENV{ROS_DISTRO} MATCHES "galactic")
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-ignored-qualifiers)
endif()

find_package(ament_cmake REQUIRED)
find_package(fkie_iop_events REQUIRED)
find_package(fkie_iop_msgs REQUIRED)
find_package(std_srvs REQUIRED)

###############
## setup IOP ##
###############
iop_init()
iop_export_service(
  urn_jaus_jss_core_Discovery
  urn_jaus_jss_core_DiscoveryClient
)
iop_code_generator(
  IDLS
    urn.jaus.jss.core-v1.1/Events.xml
    urn.jaus.jss.core-v1.1/EventsClient.xml
    urn.jaus.jss.core-v1.1/Transport.xml
    urn.jaus.jss.core-v1.1/Discovery.xml
    urn.jaus.jss.core-v1.1/DiscoveryClient.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_core_Discovery/Discovery_ReceiveFSM.h
    include/urn_jaus_jss_core_Discovery/DiscoveryService.h
    include/urn_jaus_jss_core_DiscoveryClient/DiscoveryClient_ReceiveFSM.h
    include/urn_jaus_jss_core_DiscoveryClient/DiscoveryClientService.h
    src/urn_jaus_jss_core_Discovery/Discovery_ReceiveFSM.cpp
    src/urn_jaus_jss_core_DiscoveryClient/DiscoveryClient_ReceiveFSM.cpp
  EXTERN_SERVICES
    urn_jaus_jss_core_Events
    urn_jaus_jss_core_EventsClient
    urn_jaus_jss_core_Transport
  GENERATED_SOURCES cppfiles
)

###########
## Build ##
###########
set(dependencies
  fkie_iop_events
  fkie_iop_msgs
  std_srvs
)

include_directories(
  include
  ${IOP_COMPONENT_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
            src/DiscoveryComponent.cpp
            src/DiscoveryComponentList.cpp
            src/DiscoveryRosInterface.cpp
            src/DiscoveryServiceDef.cpp
            src/DiscoveryServiceList.cpp
            ${cppfiles})

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

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include ${IOP_COMPONENT_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(fkie_iop_component plugin_iop.xml)

ament_package()
