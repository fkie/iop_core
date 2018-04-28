include(CMakeParseArguments)

get_filename_component(JTS_CMAKE_SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
if(NOT EXISTS "${JTS_CMAKE_SCRIPT_DIR}/update_cmake_config.py")
  set(JTS_CMAKE_SCRIPT_DIR "@CMAKE_CURRENT_SOURCE_DIR@/cmake")
endif()

message(STATUS "JTS: Current dir ${CMAKE_CURRENT_BINARY_DIR}")
message(STATUS "JTS: Scripts dir ${JTS_CMAKE_SCRIPT_DIR}")
set(JTS_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/build")
message(STATUS "JTS: Build dir ${JTS_BUILD_DIR}")

set(RUN_UPDATES "enabled")
if ($ENV{RUN_UPDATES})
  set(RUN_UPDATES $ENV{RUN_UPDATES})
endif()
message(STATUS "JTS: RUN_UPDATES=${RUN_UPDATES}")
file (STRINGS "${JTS_CMAKE_SCRIPT_DIR}/jts_commit" GIT_HASH)
message(STATUS "JTS: git hash: ${GIT_HASH}")

set(ENV{JTS_COMMON_PATH} "${JTS_BUILD_DIR}/jaustoolset/GUI/templates/Common")

if(NOT EXISTS "${JTS_BUILD_DIR}/jaustoolset")
  file(MAKE_DIRECTORY "${JTS_BUILD_DIR}/jaustoolset")
  message(STATUS "JTS: Clone https://github.com/fkie-forks/jaustoolset.git into jaustoolset/build")
  execute_process(
    COMMAND git clone https://github.com/fkie-forks/jaustoolset.git "${JTS_BUILD_DIR}/jaustoolset"
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
    COMMAND set -e && git -C "${JTS_BUILD_DIR}/jaustoolset" checkout $GIT_HASH
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
    WORKING_DIRECTORY "${JTS_BUILD_DIR}/jaustoolset/GUI"
  )
  message(WARNING "JTS: build result: ${result}: ${output}")
  message(STATUS "JTS: build nodeManager")
  execute_process(
    COMMAND scons
    OUTPUT_VARIABLE output
    WORKING_DIRECTORY "${JTS_BUILD_DIR}/jaustoolset/nodeManager"
  )
elseif (${RUN_UPDATES} STREQUAL "enabled")
  set(DO_BUILD "nobuild")
  execute_process(
    COMMAND git -C ${JTS_BUILD_DIR}/jaustoolset rev-parse HEAD
    OUTPUT_VARIABLE CURR_HASH
    WORKING_DIRECTORY "${JTS_BUILD_DIR}/jaustoolset"
  )
  string(STRIP ${CURR_HASH} CURR_HASH)
  message(STATUS "JTS: Current commit: : ${CURR_HASH}")
  message(STATUS "JTS: New commit: : ${GIT_HASH}")
  if (NOT ${CURR_HASH} STREQUAL ${GIT_HASH})
    execute_process(
      COMMAND git -C ${JTS_BUILD_DIR}/jaustoolset fetch
      OUTPUT_VARIABLE output
      ERROR_VARIABLE output
      RESULT_VARIABLE result
      WORKING_DIRECTORY "${JTS_BUILD_DIR}/jaustoolset"
    )
    set(DO_BUILD "build")
  endif()
  execute_process(
    COMMAND git -C ${JTS_BUILD_DIR}/jaustoolset diff ${GIT_HASH}
    OUTPUT_VARIABLE DIFF_RESULT
    WORKING_DIRECTORY "${JTS_BUILD_DIR}/jaustoolset"
  )
  if (${DIFF_RESULT})
    message(STATUS "JTS: Update JTS to commit: : ${GIT_HASH}")
    set(DO_BUILD "build")
  endif()
  if (${DO_BUILD} STREQUAL "build")
    message(STATUS "JTS: build")
    execute_process(
      COMMAND ant compile
      OUTPUT_VARIABLE output
      ERROR_VARIABLE output
      RESULT_VARIABLE result
      WORKING_DIRECTORY "${JTS_BUILD_DIR}/jaustoolset/GUI"
    )
    message(WARNING "JTS: buil result: ${result}: ${output}")
    message(STATUS "JTS: build nodeManager")
    execute_process(
      COMMAND scons
      OUTPUT_VARIABLE output
      WORKING_DIRECTORY "${JTS_BUILD_DIR}/jaustoolset/nodeManager"
    )
  endif()
  
endif()

