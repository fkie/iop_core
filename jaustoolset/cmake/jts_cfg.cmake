include(CMakeParseArguments)

if ( jaustoolset_INCLUDE_DIRS )
    if ( NOT EXISTS ${CMAKE_INSTALL_PREFIX} )
        # working directory in develspace
        list(GET jaustoolset_INCLUDE_DIRS 0 FIRST_INCLUDE)
        get_filename_component(jaustoolset_WORKING_DIR ${FIRST_INCLUDE} PATH)
    else()
        # working directory in installspace
        #get_working_dirs(jaustoolset_WORKING_DIRS)
        list(GET jaustoolset_INCLUDE_DIRS 0 FIRST_INCLUDE)
        get_filename_component(jaustoolset_INSTALL_PATH ${FIRST_INCLUDE} PATH)
        set(jaustoolset_WORKING_DIR ${jaustoolset_INSTALL_PATH}/share/jaustoolset)
    endif()
endif()

set(jaustoolset_INCLUDE_JARS_LT11
    ${jaustoolset_WORKING_DIR}/libs_external/jargs-1.0/jargs.jar:${jaustoolset_WORKING_DIR}/libs_external/runtime/commons-lang-2.5.jar:${jaustoolset_WORKING_DIR}/libs_external/smc/Smc.jar
)
set(jaustoolset_INCLUDE_JARS_S11
    ${jaustoolset_WORKING_DIR}/libs_external/jargs-1.0/jargs.jar:${jaustoolset_WORKING_DIR}/libs_external/runtime/commons-lang-2.5.jar:${jaustoolset_WORKING_DIR}/libs_external/smc/Smc.jar:${jaustoolset_WORKING_DIR}/libs_external/jaxb/javax.activation-api-1.2.0:${jaustoolset_WORKING_DIR}/libs_external/jaxb/jaxb-api-2.4.0-b180830.0359.jar:${jaustoolset_WORKING_DIR}/libs_external/jaxb/jaxb-core-2.3.0.1.jar:${jaustoolset_WORKING_DIR}/libs_external/jaxb/jaxb-impl-2.4.0-b180830.0438.jar
)

set(INCLUDE_LOCAL_JARS_LT11
    libs_external/jargs-1.0/jargs.jar
    libs_external/runtime/commons-lang-2.5.jar
    libs_external/smc/Smc.jar
)

set(INCLUDE_LOCAL_JARS_S11
    libs_external/jargs-1.0/jargs.jar
    libs_external/runtime/commons-lang-2.5.jar
    libs_external/smc/Smc.jar
    libs_external/jaxb/javax.activation-api-1.2.0.jar
    libs_external/jaxb/jaxb-api-2.4.0-b180830.0359.jar
    libs_external/jaxb/jaxb-core-2.3.0.1.jar
    libs_external/jaxb/jaxb-impl-2.4.0-b180830.0438.jar
)
