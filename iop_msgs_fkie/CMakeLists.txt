cmake_minimum_required(VERSION 2.8.3)
project(iop_msgs_fkie)

find_package(catkin REQUIRED COMPONENTS
             message_generation
             std_msgs
             )

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Component.msg
  DigitalResourceEndpoint.msg
  DigitalResourceEndpoints.msg
  HandoffRequest.msg
  HandoffResponse.msg
  Identification.msg
  JausAddress.msg
  Measurement.msg
  MeasurementValue.msg
  Node.msg
  OcuCmd.msg
  OcuCmdEntry.msg
  OcuFeedback.msg
  OcuServiceInfo.msg
  Service.msg
  Subsystem.msg
  System.msg
  VisualSensorName.msg
  VisualSensorNames.msg
)

add_service_files(
  FILES
  QueryByAddr.srv
  QueryIdentification.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
                  std_msgs
                  )

catkin_package(CATKIN_DEPENDS
               message_runtime
               std_msgs
               )
