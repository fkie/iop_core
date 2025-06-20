## How to create your own plugin for ROS/IOP-Bridge

Let's see all the steps we need to create a plugin to use within ROS/IOP-Bridge. For this example we take an existing service which offers own functionality and also exports a library which is used by other services: *urn:jaus:jss:environmentSensing:VisualSensor*.

First of all we need an JSIDL which describes the service with all input/output messages and protocol behaviour. You find these definitions in the JAUS standard, in JAUS Toolset (GUI/resources/xml/.) or in `fkie_iop_builder/jsidl/`. The definitions in `fkie_iop_builder/jsidl/` are copies from JTS. The JSIDL files already used in plugins were modified to pass the received message as argument to the service handler:
```xml
    <action name="accessControl.events.transport.Send">             -->  <action name="sendReportVisualSensorCapabilities">
       <argument value=" 'ReportVisualSensorCapabilities' "/>       -->  <argument value="msg"/>
       <argument value="transportData"/>
    </action>
```

Now we create a new ROS package (in our case *fkie_iop_visual_sensor*) which depends on `fkie_iop_component` and `fkie_iop_accesscontrol`. Each plugin should depend on `fkie_iop_component` to get the funtionatlities of [Bridge-Plugins](../fkie_iop_component/README.md) and [Builder-package](../fkie_iop_builder/README.md). The `fkie_iop_accesscontrol` package is included, because the *VisualSensor* inherits from *AccessControl* service. Our depends in package.xml looks now like:
```xml
    <buildtool_depend>ament_cmake</buildtool_depend>
    <depend>fkie_iop_accesscontrol</depend>
```

### Generate and include JTS source

In the next step we add JAUS services to `CMakeLists.txt`:
```makefile
    iop_init()
    iop_export_service(
      urn_jaus_jss_environmentSensing_VisualSensor
    )
    iop_code_generator(
      IDLS
        urn.jaus.jss.core-v1.0/AccessControl.xml
        urn.jaus.jss.core-v1.0/Events.xml
        urn.jaus.jss.core-v1.0/Transport.xml
        urn.jaus.jss.environmentSensing/VisualSensor.xml
      OWN_IDLS
      OVERRIDES
      EXTERN_SERVICES
        urn_jaus_jss_core_Events
        urn_jaus_jss_core_AccessControl
        urn_jaus_jss_core_Transport
      GENERATED_SOURCES cppfiles
    )
```
The call `iop_init()` is for all plugins the same. `iop_export_service()` specifies the service to export, so it can be included in other services. In `iop_code_generator::IDLS` we list all JAUS services included in this plugin. It is our service and all services in the inherit tree. The services we inherit from are already implemented in other packages and we want use them. To do this we add this services to `iop_code_generator::EXTERN_SERVICES`. Thereby we use the directory name created by JTS for these services.
> ! internally our build script deletes all generated sources defined in `EXTERN_SERVICES` after creation by JTS.

Now we open a terminal and go to our package and run `catkin build --this`. This generates in `build/fkie_iop_visual_sensor/jaus/fkie_iop_visual_sensor` the source files for our service. In order to extend the functionality of the generated service, we copy the following files into our package, respecting the directory structure:
```makefile
    include/urn_jaus_jss_environmentSensing_VisualSensor/VisualSensor_ReceiveFSM.h
    src/urn_jaus_jss_environmentSensing_VisualSensor/VisualSensor_ReceiveFSM.cpp
```
These files we add to `iop_code_generator::OVERRIDES` in `CMakeLists.txt`:
```makefile
    iop_code_generator(
      :
      OVERRIDES
        include/urn_jaus_jss_environmentSensing_VisualSensor/VisualSensor_ReceiveFSM.h
        src/urn_jaus_jss_environmentSensing_VisualSensor/VisualSensor_ReceiveFSM.cpp
      :
    )
```

Now we can implement the functionality for the service by extending the two files.


### Add plugin functionality

Additionally we have to add the plugin functionality!
The needed files are generated by jaustoolset. You need to copy the `plugin_iop.xml` from `build/fkie_iop_visual_sensor/jaus/fkie_iop_visual_sensor` and add this file to CMakefiles.txt:
```makefile
  pluginlib_export_plugin_description_file(fkie_iop_component plugin_iop.xml)
```

### Provide colcon specific configuration `CMakeLists.txt`
Set include directories:
```makefile
include_directories(
  include
  ${IOP_COMPONENT_INCLUDE_DIRS}
)
```

Add source files to the library:
```makefile
add_library(${PROJECT_NAME} SHARED
  ${cppfiles}
)
```

Specify libraries to link a library against:
```makefile
ament_target_dependencies(
  ${PROJECT_NAME}
  fkie_iop_accesscontrol
)
```

The service of this package is used by other packages, like *fkie_iop_digital_video* or *fkie_iop_still_image*. So that catkin works properly we have to configure `catkin_package`:
```makefile
ament_export_include_directories(include ${IOP_COMPONENT_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(fkie_iop_accesscontrol)

ament_package()
```

If the package will be used as installed package we have to add install directives:

```makefile
  # Mark IOP include files for installation
  install(
    DIRECTORY ${IOP_INSTALL_INCLUDE_DIRS}
    DESTINATION include/${PROJECT_NAME}
    PATTERN "*.old" EXCLUDE
    PATTERN "*.gen" EXCLUDE
  )

  # Mark executables and/or libraries for installation
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

```

done! Build the package with `colcon build fkie_iop_visual_sensor`