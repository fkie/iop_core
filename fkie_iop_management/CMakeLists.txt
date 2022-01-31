cmake_minimum_required(VERSION 3.5)
project(fkie_iop_management)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-ignored-qualifiers)
endif()

find_package(ament_cmake REQUIRED)
find_package(fkie_iop_accesscontrol REQUIRED)

###############
## setup IOP ##
###############
iop_init()
iop_export_service(
    urn_jaus_jss_core_Management
    urn_jaus_jss_core_ManagementClient
)
iop_code_generator(
  IDLS
    urn.jaus.jss.core-v1.1/Events.xml
    urn.jaus.jss.core-v1.1/EventsClient.xml
    urn.jaus.jss.core-v1.1/Transport.xml
    urn.jaus.jss.core-v1.1/AccessControl.xml
    urn.jaus.jss.core-v1.1/AccessControlClient.xml
    urn.jaus.jss.core-v1.1/Management.xml
    urn.jaus.jss.core-v1.1/ManagementClient.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_core_Management/Management_ReceiveFSM.h
    include/urn_jaus_jss_core_ManagementClient/ManagementClient_ReceiveFSM.h
    src/urn_jaus_jss_core_Management/Management_ReceiveFSM.cpp
    src/urn_jaus_jss_core_ManagementClient/ManagementClient_ReceiveFSM.cpp
  EXTERN_SERVICES
      urn_jaus_jss_core_AccessControl
      urn_jaus_jss_core_AccessControlClient
      urn_jaus_jss_core_Events
      urn_jaus_jss_core_EventsClient
      urn_jaus_jss_core_Transport
  GENERATED_SOURCES cppfiles
)

###########
## Build ##
###########
set(dependencies
  fkie_iop_accesscontrol
)

include_directories(
  include
  ${IOP_COMPONENT_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
            ${cppfiles}
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

# install(
#   DIRECTORY include/${PROJECT_NAME}
#   DESTINATION include
#   PATTERN "*.old" EXCLUDE
#   PATTERN "*.gen" EXCLUDE
# )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include ${IOP_COMPONENT_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(fkie_iop_component plugin_iop.xml)

ament_package()
