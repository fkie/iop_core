cmake_minimum_required(VERSION 2.8.3)
project(jaustoolset)

find_package(catkin REQUIRED COMPONENTS roscpp)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CFG_EXTRAS jts_cfg.cmake
)

###################################
## build jaustoolset library     ##
###################################
include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)
add_library(${PROJECT_NAME}
    src/EventReceiver.cpp
    src/InternalEvents/InternalEventHandler.cpp
    src/InternalEvents/Receive.cpp
    src/InternalEvents/Receive_1_0.cpp
    src/InternalEvents/Receive_1_1.cpp
    src/InternalEvents/Send.cpp
    src/InternalEvents/Send_1_0.cpp
    src/InternalEvents/Send_1_1.cpp
    src/Messages/MessageTransport.cpp
    src/Service.cpp
    src/SimpleThread.cpp
    src/Transport/Address.cpp
    src/Transport/ChecksumCRC.cpp
    src/Transport/JSerial.cpp
    src/Transport/JTCPTransport.cpp
    src/Transport/JUDPTransport.cpp
    src/Transport/JUDPTransportLB.cpp
    src/Transport/JausAddress.cpp
    src/Transport/JausTransport.cpp
    src/Transport/JrLogger.cpp
    src/Transport/JrSockets.cpp
    src/Transport/JuniorAPI.cpp
    src/Transport/JuniorMgr.cpp
    src/Transport/JuniorRTE.cpp
    src/Transport/OS.cpp
    src/Transport/TCPConnection.cpp
    src/Transport/XmlConfig.cpp
    src/Transport/tinyxml/tinyxml.cpp
    src/Transport/tinyxml/tinyxmlerror.cpp
    src/Transport/tinyxml/tinyxmlparser.cpp
    src/jFixedLengthString.cpp
)

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${RT_LIBRARY}
)

###################################
## build JTSNodeManager          ##
###################################

add_executable(JTSNodeManager src/jts_node_manager.cpp)
target_link_libraries(JTSNodeManager
                      ${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${RT_LIBRARY}
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

# include java.se.ee in java version 9
#if (${Java_VERSION_MAJOR} EQUAL "9" OR ${Java_VERSION_MAJOR} EQUAL "10")
if (${Java_VERSION_MAJOR} MATCHES "^(9|10)$")
    set(CMAKE_JAVA_COMPILE_FLAGS --add-modules java.xml.bind)
endif()

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

add_jar(iop_codegenerator ${JAVA_SOURCE_FILES}
    INCLUDE_JARS ${INCLUDE_LOCAL_JARS}
    ENTRY_POINT org/jts/codegenerator/CodeGenerator
    OUTPUT_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
)

###################################
## install iop_codegenerator.jar ##
###################################
install_jar(iop_codegenerator ${CATKIN_PACKAGE_LIB_DESTINATION})

###############################################################
## install resources and specific libs for iop_codegenerator ##
###############################################################
install(
  DIRECTORY libs_external DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY templates DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


################################################
## install includes, library and node manager ##
################################################
install(
  DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} JTSNodeManager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
    PROGRAMS
        scripts/jaus_node_manager.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

