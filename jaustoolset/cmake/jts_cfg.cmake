include(CMakeParseArguments)

set(jaustoolset_SHARE_PATH ${CMAKE_INSTALL_PREFIX}/share/jaustoolset)

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()    

set(jaustoolset_INCLUDE_JARS_LT11
    ${jaustoolset_SHARE_PATH}/libs_external/jargs-1.0/jargs.jar:${jaustoolset_SHARE_PATH}/libs_external/runtime/commons-lang-2.5.jar:${jaustoolset_SHARE_PATH}/libs_external/smc/Smc.jar
)
set(jaustoolset_INCLUDE_JARS_S11
    ${jaustoolset_SHARE_PATH}/libs_external/jargs-1.0/jargs.jar:${jaustoolset_SHARE_PATH}/libs_external/runtime/commons-lang-2.5.jar:${jaustoolset_SHARE_PATH}/libs_external/smc/Smc.jar:${jaustoolset_SHARE_PATH}/libs_external/jaxb/javax.activation-api-1.2.0:${jaustoolset_SHARE_PATH}/libs_external/jaxb/jaxb-api-2.4.0-b180830.0359.jar:${jaustoolset_SHARE_PATH}/libs_external/jaxb/jaxb-core-2.3.0.1.jar:${jaustoolset_SHARE_PATH}/libs_external/jaxb/jaxb-impl-2.4.0-b180830.0438.jar
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
