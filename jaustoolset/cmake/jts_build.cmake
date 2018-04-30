include(CMakeParseArguments)

get_filename_component(JTS_CMAKE_SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
if(NOT EXISTS "${JTS_CMAKE_SCRIPT_DIR}/update_cmake_config.py")
  set(JTS_CMAKE_SCRIPT_DIR "@CMAKE_CURRENT_SOURCE_DIR@/cmake")
endif()

message(STATUS "JTS: Current dir ${CMAKE_CURRENT_BINARY_DIR}")
message(STATUS "JTS: Scripts dir ${JTS_CMAKE_SCRIPT_DIR}")
set(JTS_DIR "${CMAKE_CURRENT_BINARY_DIR}/../../jaustoolset/")
get_filename_component(JTS_DIR ${JTS_DIR} ABSOLUTE)
message(STATUS "JTS: path ${JTS_DIR}")
set(JTS_GUI_DIR "${JTS_DIR}/GUI")
# set JTS environment variable needed by all depended packages
set(ENV{JTS_COMMON_PATH} "${JTS_DIR}/GUI/templates/Common")
message(STATUS "JTS: export JTS_COMMON_PATH=$ENV{JTS_COMMON_PATH}")
# should we update jaustoolset?
set(RUN_UPDATES "enabled")
if ($ENV{RUN_UPDATES})
  set(RUN_UPDATES $ENV{RUN_UPDATES})
endif()
message(STATUS "JTS: RUN_UPDATES=${RUN_UPDATES}")
# read current commit hash
file (STRINGS "${JTS_CMAKE_SCRIPT_DIR}/jts_commit" GIT_HASH)
message(STATUS "JTS: git hash: ${GIT_HASH}")

# print macro
macro(print msg result output)
  if (${result} GREATER "0")
    message(WARNING "JTS: Error on ${msg}: ${result}: ${output}")
  else()
    message(STATUS "JTS: ${msg}: ${output}")
  endif()
endmacro()


macro(prepare_jts_build)
    execute_process(
      COMMAND ant bindmxGraph -v
      OUTPUT_VARIABLE output
      ERROR_VARIABLE output
      RESULT_VARIABLE result
      WORKING_DIRECTORY "${JTS_DIR}/GUI"
    )
    print("ant bindmxGraph" ${result} ${output})
    execute_process(
      COMMAND ant bindJSIDLPlus -v
      OUTPUT_VARIABLE output
      ERROR_VARIABLE output
      RESULT_VARIABLE result
      WORKING_DIRECTORY "${JTS_DIR}/GUI"
    )
    print("ant bindJSIDLPlus" ${result} ${output})
    execute_process(
      COMMAND ant bind -v
      OUTPUT_VARIABLE output
      ERROR_VARIABLE output
      RESULT_VARIABLE result
      WORKING_DIRECTORY "${JTS_DIR}/GUI"
    )
    print("ant bind" ${result} ${output})
    execute_process(
      COMMAND ant compile-promela -v
      OUTPUT_VARIABLE output
      ERROR_VARIABLE output
      RESULT_VARIABLE result
      WORKING_DIRECTORY "${JTS_DIR}/GUI"
    )
    print("ant compile-promela" ${result} ${output})
endmacro()


macro(build_jts)
    message(STATUS "JTS: build")
    execute_process(
      COMMAND ant compile -v
      OUTPUT_VARIABLE output
      ERROR_VARIABLE output
      RESULT_VARIABLE result
      WORKING_DIRECTORY "${JTS_DIR}/GUI"
    )
    print("ant compile" ${result} ${output})
    message(STATUS "JTS: build nodeManager")
    execute_process(
      COMMAND scons
      OUTPUT_VARIABLE output
      OUTPUT_VARIABLE output
      ERROR_VARIABLE output
      RESULT_VARIABLE result
      WORKING_DIRECTORY "${JTS_DIR}/nodeManager"
    )
    print("nodeManager scons" ${result} ${output})
endmacro()


if(NOT EXISTS "${JTS_DIR}")
  message(STATUS "***************************************************************")
  message(STATUS "*                     Jaustoolset                             *")
  message(STATUS "*                                                             *")
  message(STATUS "*  On the first run, a lot of stuff will be downloaded from:  *")
  message(STATUS "*  https://github.com/fkie-forks/jaustoolset.git              *")
  message(STATUS "*  into ${JTS_DIR}")
  message(STATUS "*  This may take a while...                                   *")
  message(STATUS "*                                                             *")
  message(STATUS "***************************************************************")
  execute_process(
    COMMAND git clone https://github.com/fkie-forks/jaustoolset.git "${JTS_DIR}"
    OUTPUT_VARIABLE output
    ERROR_VARIABLE output
    RESULT_VARIABLE result
  )
  print("Clone result" ${result} ${output})
  message(STATUS "JTS: change commit to hash: ${GIT_HASH}")
  execute_process(
    COMMAND set -e && git -C "${JTS_DIR}" checkout $GIT_HASH
    OUTPUT_VARIABLE output
    ERROR_VARIABLE output
    RESULT_VARIABLE result
  )
  prepare_jts_build()
  build_jts()
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
      COMMAND git -C ${JTS_DIR} diff-index ${GIT_HASH} --
      OUTPUT_VARIABLE DIFF_RESULT
      WORKING_DIRECTORY "${JTS_DIR}"
  )
  if (NOT ${DIFF_RESULT} STREQUAL "")
    message(STATUS "JTS: Update JTS to commit: : ${GIT_HASH}")
    set(DO_BUILD "build")
  endif()
  # test for required library
  if(NOT EXISTS "${JTS_DIR}/nodeManager/bin/libCommon.so")
    message(STATUS "JTS: no libCommon library -> build")
    set(DO_BUILD "build")
    prepare_jts_build()
  endif()
  # build on local or remote changes
  if (${DO_BUILD} STREQUAL "build")
    build_jts()
  endif()

endif()

