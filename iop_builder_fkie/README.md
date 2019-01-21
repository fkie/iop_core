This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## Builder Package - iop_builder_fkie

The `iop_builder_fkie` package is the base of the ROS/IOP-Bridge. It uses the JAUS Toolset (JTS) to generate the source code from JSIDL files. The JSIDL files describe the JAUS messages and the structure of the IOP/JAUS services. They are also located in this package. In general the JSIDL file can be stored in each plugin. But it is recommended to store these in this package.

To generate sources from JSIDL files the `iop_build_fkie` extends `CMakeLists.txt` by three scripts: `iop_init`, `iop_export_service` and `iop_code_generator`. Each ROS/IOP plugin should include at least `iop_init` and `iop_code_generator` in `CMakeLists.txt` to build a plugin.



### `iop_init` Parameter description
```makefile
iop_init(COMPONENT_ID 0)
```

####  : COMPONENT_ID
This ID is used by JTS to create a component. We use the sources of this component to create our ROS/IOP-Bridge plugin. This ID is also hard coded in the `main.cpp` and generated path of the component. The auto-generated source-code files of a component are located in the `build` folder of the ROS workspace. For `iop_primitive_driver_fkie` it is `build/iop/urn.jaus.jss.mobility/iop_primitive_driver_fkie/jaus/Iop_primitive_driver_fkie_0`
> after we use only a part of JTS-sources you can set it to `0` the only effect is the generated path name in the build folder.



### `iop_export_service` Parameter description
```makefile
iop_export_service(urn_jaus_jss_core_Events urn_jaus_jss_core_EventsClient)
```
There are a lot of services with parent functionality which is inherited by other services. These are e.g. `Transport`, `Events` and more services. In this case we need to export the header files and libraries so that they can be used by other plugins. The `iop_export_service` creates a list of all header files from specified services. The services are specified as a list of names. The name is defined by auto-generated folder name for JAUS service located in `build`-folder. An example from Events service:

The header fiels are stored in `${IOP_INSTALL_INCLUDE_DIRS}` variable. We can use it to install theses files like:
```makefile
install(
  DIRECTORY ${IOP_INSTALL_INCLUDE_DIRS} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)
```
The library is exported by `catkin_package`, e.g. from Events service:
```makefile
catkin_package(
    INCLUDE_DIRS include/public
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS iop_transport_fkie
)
```



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
Here are specified all source-code files which are auto-generated and changed by the plugin. Copy the files you will change from auto-generated location in `build` folder to the own plugin and specify these here.
>New source-code file which are not generated, you have to specify in `add_library` (a default ROS parameter). See for example the CMakeLists.txt of `iop_discovery_fkie`.

>Do not put own non-generated *include* files in the top level of the `include`-folder of your plugin. Please create `public` or `private` subfolder. See for example the `iop_discovery_fkie` package.


#### : EXTERN_SERVICES
A list of IOP/JAUS services for which a library is available and will be used in our plugin. The name is defined by auto-generated folder name for JAUS service located in `build`-folder.

>The ROS packages of the used libraries must be specified in `find_package` and `catkin_package`/`CATKIN_DEPENDS` of the *CMakeLists.txt* and also in *package.xml*


#### : GENERATED_SOURCES
Since the auto-generated source-code files are also compiled, this is the return of `iop_code_generator`. The list of returned files can now be used by name *cppfiles* in `add_library`, e.g.
```makefile
add_library(${PROJECT_NAME}
            ${cppfiles})
```
