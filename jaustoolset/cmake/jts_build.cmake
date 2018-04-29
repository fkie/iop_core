include(CMakeParseArguments)

get_filename_component(JTS_CMAKE_SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
if(NOT EXISTS "${JTS_CMAKE_SCRIPT_DIR}/update_cmake_config.py")
  set(JTS_CMAKE_SCRIPT_DIR "@CMAKE_CURRENT_SOURCE_DIR@/cmake")
endif()

message(STATUS "JTS: Current dir ${CMAKE_CURRENT_BINARY_DIR}")
message(STATUS "JTS: Scripts dir ${JTS_CMAKE_SCRIPT_DIR}")
set(JTS_DIR "${CMAKE_CURRENT_BINARY_DIR}/../../jaustoolset/")
message(STATUS "JTS: path ${JTS_DIR}")
set(JTS_GUI_DIR "${JTS_DIR}/GUI")

set(RUN_UPDATES "enabled")
if ($ENV{RUN_UPDATES})
  set(RUN_UPDATES $ENV{RUN_UPDATES})
endif()
message(STATUS "JTS: RUN_UPDATES=${RUN_UPDATES}")
file (STRINGS "${JTS_CMAKE_SCRIPT_DIR}/jts_commit" GIT_HASH)
message(STATUS "JTS: git hash: ${GIT_HASH}")

# set JTS environment variable needed by all depended packages
set(ENV{JTS_COMMON_PATH} "${JTS_DIR}/GUI/templates/Common")
# to be able to build in travis - docker
message(STATUS "ENV: ${ENV}")
message(STATUS "JAVA_HOME: $ENV{JAVA_HOME}")
#unset(ENV{JAVA_HOME})
message(STATUS "JAVA_HOME: $ENV{JAVA_HOME}")

if(NOT EXISTS "${JTS_DIR}")
  file(MAKE_DIRECTORY "${JTS_DIR}")
  message(STATUS "*********************************************************")
  message(STATUS "*                     Jaustoolset                       *")
  message(STATUS "*                                                       *")
  message(STATUS "*  On the first run, a lot of stuff will be downloaded  *")
  message(STATUS "*  This may take a while...                             *")
  message(STATUS "*                                                       *")
  message(STATUS "*********************************************************")

  message(STATUS "JTS: Clone https://github.com/fkie-forks/jaustoolset.git into ${JTS_DIR}")
  execute_process(
    COMMAND git clone https://github.com/fkie-forks/jaustoolset.git "${JTS_DIR}"
    OUTPUT_VARIABLE output
    ERROR_VARIABLE output
    RESULT_VARIABLE result
#    ERROR_QUIET
  )
  if (${result} GREATER "0")
    message(WARNING "JTS: Clone result: ${result}: ${output}")
  endif()
  message(STATUS "JTS: change commit to hash: ${GIT_HASH}")
  execute_process(
    COMMAND set -e && git -C "${JTS_DIR}" checkout $GIT_HASH
    OUTPUT_VARIABLE output
  )
  message(STATUS "JTS: build")
  execute_process(
    COMMAND ant bindmxGraph
    COMMAND ant bindJSIDLPlus
    COMMAND ant bind
    COMMAND ant compile-promela
    COMMAND ant compile
    OUTPUT_VARIABLE output
    ERROR_VARIABLE output
    RESULT_VARIABLE result
    WORKING_DIRECTORY "${JTS_DIR}/GUI"
  )
  message(WARNING "JTS: build result: ${result}: ${output}")
  message(STATUS "JTS: build nodeManager")
  execute_process(
    COMMAND scons
    OUTPUT_VARIABLE output
    WORKING_DIRECTORY "${JTS_DIR}/nodeManager"
  )
elseif (${RUN_UPDATES} STREQUAL "enabled")
  # jaustoolset sourcecode is downloaded, test for changes
  set(DO_BUILD "nobuild")
  # test for remote changes in jaustoolset based on hash in jts_commit
  execute_process(
    COMMAND git -C ${JTS_DIR} rev-parse HEAD
    OUTPUT_VARIABLE CURR_HASH
    WORKING_DIRECTORY "${JTS_DIR}"
  )
  string(STRIP ${CURR_HASH} CURR_HASH)
  message(STATUS "JTS: Current commit: : ${CURR_HASH}")
  message(STATUS "JTS: New commit: : ${GIT_HASH}")
  if (NOT ${CURR_HASH} STREQUAL ${GIT_HASH})
    execute_process(
      COMMAND git -C ${JTS_DIR} fetch
      OUTPUT_VARIABLE output
      ERROR_VARIABLE output
      RESULT_VARIABLE result
      WORKING_DIRECTORY "${JTS_DIR}"
    )
    set(DO_BUILD "build")
  endif()
  # test for local changes in jaustoolset
  execute_process(
    COMMAND git -C ${JTS_DIR} diff ${GIT_HASH}
    OUTPUT_VARIABLE DIFF_RESULT
    WORKING_DIRECTORY "${JTS_DIR}"
  )
  if (${DIFF_RESULT})
    message(STATUS "JTS: Update JTS to commit: : ${GIT_HASH}")
    set(DO_BUILD "build")
  endif()
  # test for required library
  if(NOT EXISTS "${JTS_DIR}/nodeManager/bin/libCommon.so")
    message(STATUS "JTS: no libCommon library -> build")
    set(DO_BUILD "build")
    execute_process(
      COMMAND ant --noconfig bindmxGraph
      COMMAND ant --noconfig bindJSIDLPlus
      COMMAND ant --noconfig bind
      COMMAND ant --noconfig compile-promela
      OUTPUT_VARIABLE output
      ERROR_VARIABLE output
      RESULT_VARIABLE result
      WORKING_DIRECTORY "${JTS_DIR}/GUI"
    )
  endif()
  # build on local or remote changes
  if (${DO_BUILD} STREQUAL "build")
    message(STATUS "JTS: build")
    execute_process(
      COMMAND ant compile
      OUTPUT_VARIABLE output
      ERROR_VARIABLE output
      RESULT_VARIABLE result
      WORKING_DIRECTORY "${JTS_DIR}/GUI"
    )
    message(WARNING "JTS: build result: ${result}: ${output}")
    message(STATUS "JTS: build nodeManager")
    execute_process(
      COMMAND scons
      OUTPUT_VARIABLE output
      WORKING_DIRECTORY "${JTS_DIR}/nodeManager"
    )
  endif()
  
endif()

