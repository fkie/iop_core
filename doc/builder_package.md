## Builder Package

The `iop_builder_fkie` package is the core of the ROS/IOP-Bridge. This package must be included in each ROS/IOP-Bridge component. It uses the JAUS Toolset to generate the source code from JSIDL files. The JSIDL files describe the JAUS messages and the structure of the IOP/JAUS services. They are also located in this package. In general the JSIDL file can be stored in each component. But it is recommended to store these in this package.

To create a component the `iop_builder_fkie` combines the JAUS services to a component. The included service are defined in `CMakeLists.txt` of each ROS/IOP-Bridge component by `iop_code_generator`. You have to specify also a component id for the component.

Parameter used (needed) by `iop_builder_fkie`:

### `iop_init` Parameter description
```makefile
iop_init(COMPONENT_ID 33)
```
####  : COMPONENT_ID
This ID is used by JTS to create a component. This number is hard coded in the `main.cpp` and generated path of the component. The auto-generated source-code files of a component are located in the `build` folder of the ROS workspace. For `iop_primitive_driver_fkie` it is `build/iop/urn.jaus.jss.mobility/iop_primitive_driver_fkie/jaus/Iop_primitive_driver_fkie_33`

### `iop_code_generator` Parameter description

```makefile
iop_code_generator(
  IDLS
    urn.jaus.jss.core-v1.0/AccessControl.xml
    urn.jaus.jss.core-v1.0/DiscoveryClient.xml
    urn.jaus.jss.core-v1.0/Events.xml
    urn.jaus.jss.core-v1.0/Liveness.xml
    urn.jaus.jss.core-v1.0/Management.xml
    urn.jaus.jss.core-v1.0/Transport.xml
    urn.jaus.jss.mobility/PrimitiveDriver.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_mobility_PrimitiveDriver/PrimitiveDriver_ReceiveFSM.h
    src/urn_jaus_jss_mobility_PrimitiveDriver/PrimitiveDriver_ReceiveFSM.cpp
    src/main.cpp
  EXTERN_SERVICES
    urn_jaus_jss_core_AccessControl
    urn_jaus_jss_core_DiscoveryClient
    urn_jaus_jss_core_Events
    urn_jaus_jss_core_Liveness
    urn_jaus_jss_core_Management
    urn_jaus_jss_core_Transport
  GENERATED_SOURCES cppfiles
)
```

#### : IDLS
Specify a list of JSIDL files with IOP/JAUS services. This path is relative to `iop_builder_fkie/jsidl`.

#### : OWN_IDLS
Specify a list of JSIDL files with IOP/JAUS services. This path is relative to own package.

#### : OVERRIDES
Here are specified all source-code files which are auto-generated and changed by the component. Copy the files you will change from auto-generated location in `build` folder to the own component and specify these here.
>New source-code file which are not generated, you have to specify in `add_library` (a default ROS parameter). See for example the CMakeLists.txt of `iop_discovery_fkie`.

>Do not put own non-generated *include* files in the top level of the `include`-folder of your component. Please create `public` or `private` subfolder. See for example the `iop_discovery_fkie` package.

#### : EXTERN_SERVICES
A list of IOP/JAUS services for which a library is available and will be used in our component. The name is defined by auto-generated folder name for JAUS service located in `build`-folder.

>The ROS packages of the used libraries must be specified in `find_package` and `catkin_package`/`CATKIN_DEPENDS` of the *CMakeLists.txt* and also in *package.xml*

>If your package creates also a library you need to export the IOP/JAUS services by `iop_export_service` e.g.
```makefile
iop_export_service(urn_jaus_jss_core_Discovery
                   urn_jaus_jss_core_DiscoveryClient)
```
and specify your package in `catkin_package`/`LIBRARIES` of the *CMakeLists.txt*

#### : GENERATED_SOURCES
Since the auto-generated source-code files are also compiled, this is the return of `iop_code_generator`. The list of returned files can now be used by name *cppfiles* in `add_library`, e.g.
```makefile
add_library(${PROJECT_NAME}
            ${cppfiles})
```
>For the compilation of the code some other variables are returned by `iop_code_generator`. These are: ${IOP_INSTALL_INCLUDE_DIRS}, ${IOP_INCLUDE_DIRS}, ${IOP_LIBRARIES}. See one of the *CMakeList.txt* for the usage of them.
