cmake_minimum_required(VERSION 3.5)
project(jaustoolset)

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
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

set(executable_name JTSNodeManager)
set(library_name ${PROJECT_NAME})
include_directories(
    include
)

###################################
## build jaustoolset library     ##
###################################

add_library(${library_name} SHARED
    src/EventReceiver.cpp
    src/InternalEvents/InternalEventHandler.cpp
    src/InternalEvents/Receive.cpp
    src/InternalEvents/Send.cpp
    src/Service.cpp
    src/SimpleThread.cpp
    src/Transport/Address.cpp
    src/Transport/ChecksumCRC.cpp
    src/Transport/JUDPTransport.cpp
    src/Transport/JausAddress.cpp
    src/Transport/JausTransport.cpp
    src/Transport/JrLogger.cpp
    src/Transport/JrSockets.cpp
    src/Transport/JuniorAPI.cpp
    src/Transport/JuniorMgr.cpp
    src/Transport/OS.cpp
    src/Transport/XmlConfig.cpp
    src/jFixedLengthString.cpp
)



# ament_target_dependencies(${library_name}
#     ${dependencies}
#     ${RT_LIBRARY}
#     #${TinyXML2_LIBRARIES}
# )

target_link_libraries(${library_name}
    pthread
    rt
    ${TinyXML2_LIBRARIES}
)

###################################
## build JTSNodeManager          ##
###################################

add_executable(${executable_name}
    src/rte/JSerial.cpp
    src/rte/JTCPTransport.cpp
    src/rte/jts_node_manager.cpp
    src/rte/JUDPTransportLB.cpp
    src/rte/JuniorRTE.cpp
    src/rte/TCPConnection.cpp
)

# ament_target_dependencies(${executable_name}
#   ${dependencies}
# # ${CMAKE_THREAD_LIBS_INIT}
# #                           ${RT_LIBRARY}
# #                       # ${PROJECT_NAME}
# #                       # ${catkin_LIBRARIES}
# #                       # ${RT_LIBRARY}
# #                       ${rclcpp_LIBRARIES}
# #                       ${TinyXML2_LIBRARIES}
# )


target_link_libraries(${executable_name} ${library_name}
    pthread
    rt
    ${TinyXML2_LIBRARIES}
)



###################################
## build iop_codegenerator.jar   ##
###################################

find_package(Java)
include(UseJava)

if (NOT ${Java_FOUND})
    message(FATAL_ERROR "java not found")
endif()
message(STATUS "use ${Java_JAVAC_EXECUTABLE} compiler")

set(JAVA_SOURCE_FILES
    src/org/jts/codegenerator/CodeGenerator.java
    src/org/jts/codegenerator/ArrayGenerator.java
    src/org/jts/codegenerator/BindingSupport.java
    src/org/jts/codegenerator/BitFieldGenerator.java
    src/org/jts/codegenerator/BodyGenerator.java
    src/org/jts/codegenerator/CodeGeneratorException.java
    src/org/jts/codegenerator/CodeLines.java
    src/org/jts/codegenerator/CodeWriter.java
    src/org/jts/codegenerator/CommentWriter.java
    src/org/jts/codegenerator/ComponentGenerator.java
    src/org/jts/codegenerator/ConstantsGenerator.java
    src/org/jts/codegenerator/CppCode.java
    src/org/jts/codegenerator/CSharpCode.java
    src/org/jts/codegenerator/EventDefGenerator.java
    src/org/jts/codegenerator/FixedFieldGenerator.java
    src/org/jts/codegenerator/FixedLengthStringGenerator.java
    src/org/jts/codegenerator/FooterGenerator.java
    src/org/jts/codegenerator/HeaderGenerator.java
    src/org/jts/codegenerator/InternalEventsSetGenerator.java
    src/org/jts/codegenerator/JavaCode.java
    src/org/jts/codegenerator/ListGenerator.java
    src/org/jts/codegenerator/MessageDefGenerator.java
    src/org/jts/codegenerator/MessageSetGenerator.java
    src/org/jts/codegenerator/Pair.java
    src/org/jts/codegenerator/PresenceVectorGenerator.java
    src/org/jts/codegenerator/protocolBehavior/Action.java
    src/org/jts/codegenerator/protocolBehavior/Entry.java
    src/org/jts/codegenerator/protocolBehavior/Exit.java
    src/org/jts/codegenerator/protocolBehavior/GenericState.java
    src/org/jts/codegenerator/protocolBehavior/Guard.java
    src/org/jts/codegenerator/protocolBehavior/Parameter.java
    src/org/jts/codegenerator/protocolBehavior/State.java
    src/org/jts/codegenerator/protocolBehavior/StateWrapper.java
    src/org/jts/codegenerator/protocolBehavior/Transition.java
    src/org/jts/codegenerator/ProtocolBehaviorGenerator.java
    src/org/jts/codegenerator/RecordGenerator.java
    src/org/jts/codegenerator/ScaleRangeGenerator.java
    src/org/jts/codegenerator/SequenceGenerator.java
    src/org/jts/codegenerator/ServiceDefGenerator.java
    src/org/jts/codegenerator/ServiceSetResolver.java
    src/org/jts/codegenerator/support/AeSimpleSHA1.java
    src/org/jts/codegenerator/support/ClassDeclaration.java
    src/org/jts/codegenerator/support/EMemberVisibility.java
    src/org/jts/codegenerator/support/FieldDeclaration.java
    src/org/jts/codegenerator/support/FileExtensionFilter.java
    src/org/jts/codegenerator/support/FileFilter.java
    src/org/jts/codegenerator/support/InheritanceHelper.java
    src/org/jts/codegenerator/support/IVisibility.java
    src/org/jts/codegenerator/support/MethodDeclaration.java
    src/org/jts/codegenerator/support/ParameterDeclaration.java
    src/org/jts/codegenerator/support/PathExtender.java
    src/org/jts/codegenerator/support/Reference.java
    src/org/jts/codegenerator/support/ReferenceHolder.java
    src/org/jts/codegenerator/support/SconstructGenerator.java
    src/org/jts/codegenerator/support/TemplateHandler.java
    src/org/jts/codegenerator/support/VisualStudioGenerator.java
    src/org/jts/codegenerator/Util.java
    src/org/jts/codegenerator/ValueSetGenerator.java
    src/org/jts/codegenerator/VariableFieldGenerator.java
    src/org/jts/codegenerator/VariableFormatFieldGenerator.java
    src/org/jts/codegenerator/VariableLengthFieldGenerator.java
    src/org/jts/codegenerator/VariableLengthStringGenerator.java
    src/org/jts/codegenerator/VariantGenerator.java
    src/org/jts/jsidl/binding/Action.java
    src/org/jts/jsidl/binding/Argument.java
    src/org/jts/jsidl/binding/Array.java
    src/org/jts/jsidl/binding/Assumptions.java
    src/org/jts/jsidl/binding/BitField.java
    src/org/jts/jsidl/binding/BitRange.java
    src/org/jts/jsidl/binding/Body.java
    src/org/jts/jsidl/binding/ClientOf.java
    src/org/jts/jsidl/binding/ConstDef.java
    src/org/jts/jsidl/binding/CountField.java
    src/org/jts/jsidl/binding/DeclaredArray.java
    src/org/jts/jsidl/binding/DeclaredBitField.java
    src/org/jts/jsidl/binding/DeclaredBody.java
    src/org/jts/jsidl/binding/DeclaredConstSet.java
    src/org/jts/jsidl/binding/DeclaredConstSetRef.java
    src/org/jts/jsidl/binding/DeclaredEventDef.java
    src/org/jts/jsidl/binding/DeclaredFixedField.java
    src/org/jts/jsidl/binding/DeclaredFixedLengthString.java
    src/org/jts/jsidl/binding/DeclaredFooter.java
    src/org/jts/jsidl/binding/DeclaredHeader.java
    src/org/jts/jsidl/binding/DeclaredList.java
    src/org/jts/jsidl/binding/DeclaredMessageDef.java
    src/org/jts/jsidl/binding/DeclaredRecord.java
    src/org/jts/jsidl/binding/DeclaredSequence.java
    src/org/jts/jsidl/binding/DeclaredTypeSet.java
    src/org/jts/jsidl/binding/DeclaredTypeSetRef.java
    src/org/jts/jsidl/binding/DeclaredVariableField.java
    src/org/jts/jsidl/binding/DeclaredVariableFormatField.java
    src/org/jts/jsidl/binding/DeclaredVariableLengthField.java
    src/org/jts/jsidl/binding/DeclaredVariableLengthString.java
    src/org/jts/jsidl/binding/DeclaredVariant.java
    src/org/jts/jsidl/binding/DefaultState.java
    src/org/jts/jsidl/binding/DefaultTransition.java
    src/org/jts/jsidl/binding/Description.java
    src/org/jts/jsidl/binding/Dimension.java
    src/org/jts/jsidl/binding/EndState.java
    src/org/jts/jsidl/binding/Entry.java
    src/org/jts/jsidl/binding/EventDef.java
    src/org/jts/jsidl/binding/Exit.java
    src/org/jts/jsidl/binding/FixedField.java
    src/org/jts/jsidl/binding/FixedLengthString.java
    src/org/jts/jsidl/binding/Footer.java
    src/org/jts/jsidl/binding/FormatEnum.java
    src/org/jts/jsidl/binding/FormatField.java
    src/org/jts/jsidl/binding/Guard.java
    src/org/jts/jsidl/binding/Header.java
    src/org/jts/jsidl/binding/InheritsFrom.java
    src/org/jts/jsidl/binding/InputSet.java
    src/org/jts/jsidl/binding/Internal.java
    src/org/jts/jsidl/binding/InternalEventsSet.java
    src/org/jts/jsidl/binding/List.java
    src/org/jts/jsidl/binding/MessageDef.java
    src/org/jts/jsidl/binding/MessageSet.java
    src/org/jts/jsidl/binding/MxArray.java
    src/org/jts/jsidl/binding/MxCell.java
    src/org/jts/jsidl/binding/MxGeometry.java
    src/org/jts/jsidl/binding/MxPoint.java
    src/org/jts/jsidl/binding/MxRectangle.java
    src/org/jts/jsidl/binding/ObjectFactory.java
    src/org/jts/jsidl/binding/OutputSet.java
    src/org/jts/jsidl/binding/package-info.java
    src/org/jts/jsidl/binding/Parameter.java
    src/org/jts/jsidl/binding/Pop.java
    src/org/jts/jsidl/binding/PresenceVector.java
    src/org/jts/jsidl/binding/ProtocolBehavior.java
    src/org/jts/jsidl/binding/PseudoStartState.java
    src/org/jts/jsidl/binding/Push.java
    src/org/jts/jsidl/binding/Record.java
    src/org/jts/jsidl/binding/References.java
    src/org/jts/jsidl/binding/ScaleRange.java
    src/org/jts/jsidl/binding/SendAction.java
    src/org/jts/jsidl/binding/Sequence.java
    src/org/jts/jsidl/binding/ServiceDef.java
    src/org/jts/jsidl/binding/ServiceSet.java
    src/org/jts/jsidl/binding/Simple.java
    src/org/jts/jsidl/binding/Start.java
    src/org/jts/jsidl/binding/State.java
    src/org/jts/jsidl/binding/StateMachine.java
    src/org/jts/jsidl/binding/SubField.java
    src/org/jts/jsidl/binding/Transition.java
    src/org/jts/jsidl/binding/TypeAndUnitsEnum.java
    src/org/jts/jsidl/binding/TypeAndUnitsField.java
    src/org/jts/jsidl/binding/ValueEnum.java
    src/org/jts/jsidl/binding/ValueRange.java
    src/org/jts/jsidl/binding/ValueSet.java
    src/org/jts/jsidl/binding/VariableField.java
    src/org/jts/jsidl/binding/VariableFormatField.java
    src/org/jts/jsidl/binding/VariableLengthField.java
    src/org/jts/jsidl/binding/VariableLengthString.java
    src/org/jts/jsidl/binding/Variant.java
    src/org/jts/jsidl/binding/VtagField.java
)
# some variables like INCLUDE_LOCAL_JARS
include(cmake/jts_cfg.cmake)

if ("${Java_VERSION_MAJOR}" MATCHES "^(9|10)$")
    # the module JAXB marked as deprecated since java 9
    set(CMAKE_JAVA_COMPILE_FLAGS --add-modules java.xml.bind)
    set(INCLUDE_LOCAL_JARS ${INCLUDE_LOCAL_JARS_LT11})
elseif ("${Java_VERSION_MAJOR}" MATCHES "^(6|7|8)$")
    set(INCLUDE_LOCAL_JARS ${INCLUDE_LOCAL_JARS_LT11})
else ()
    # the module JAXB removed from JDK since java 11
    # use own libraries
    set(INCLUDE_LOCAL_JARS ${INCLUDE_LOCAL_JARS_S11})
endif()

add_jar(iop_codegenerator ${JAVA_SOURCE_FILES}
    INCLUDE_JARS ${INCLUDE_LOCAL_JARS}
    ENTRY_POINT org/jts/codegenerator/CodeGenerator
    OUTPUT_DIR ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
)

###################################
## install iop_codegenerator.jar ##
###################################
install_jar(iop_codegenerator ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})


###############################################################
## install resources and specific libs for iop_codegenerator ##
###############################################################
install(
  DIRECTORY libs_external
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY templates
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)


################################################
## install includes, library and node manager ##
################################################
# install(
#   DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
# )

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

# install(TARGETS JTSNodeManager ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

install(
    TARGETS ${library_name} ${executable_name} 
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    PROGRAMS
        scripts/jaus_node_manager.sh
    DESTINATION bin
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package(
    #INCLUDE_DIRS include
    CONFIG_EXTRAS cmake/jts_cfg.cmake
    #CONFIG_EXTRAS_POST
)
