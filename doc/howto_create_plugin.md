## Howto create your own plugin for ROS/IOP-Bridge

Let's see all the steps we need to create a plugin to use within ROS/IOP-Bridge. For this example we take an existing service which offers own functionality and also exports a library which is used by other services: *urn:jaus:jss:environmentSensing:VisualSensor*.

First of all we need an JSIDL which describes the service with all input/output messages and protocol behaviour. You find these definitions in the JAUS standard, in JAUS Toolset (GUI/resources/xml/.) or in `iop_builder_fkie/jsidl/`. The definitions in `iop_builder_fkie/jsidl/` are copies from JTS. The JSIDL files already used in plugins were modified to pass the received message as argument to the service handler:
```
    <action name="accessControl.events.transport.Send">             -->  <action name="sendReportVisualSensorCapabilities">
       <argument value=" 'ReportVisualSensorCapabilities' "/>       -->  <argument value="msg"/>
       <argument value="transportData"/>
    </action>
```

Now we create a new ROS package (in our case *iop_visual_sensor_fkie*) which depends on `roscpp`, `iop_component_fkie` and `iop_accesscontrol_fkie`. Each plugin should depend on `iop_component_fkie` to get the funtionatlities of [Bridge-Plugins](component_package.md) and [Builder-package](builder_package.md). The `iop_accesscontrol_fkie` package is included, because the *VisualSensor* inherits from *AccessControl* service. Our depends in package.xml looks now like:
```
    <buildtool_depend>catkin</buildtool_depend>
    <build_depend>roscpp</build_depend>
    <build_depend>iop_accesscontrol_fkie</build_depend>
    <run_depend>roscpp</run_depend>
    <run_depend>iop_accesscontrol_fkie</run_depend>
    <run_depend>iop_component_fkie</run_depend>
```

### Generate and include JTS source

In the next step we add JAUS services to `CMakeLists.txt`:
```makefile
    iop_init(COMPONENT_ID 0)
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
The call `iop_init(COMPONENT_ID 0)` can stay for all plugins the same. In `iop_code_generator::IDLS` we list all JAUS services included in this plugin. It is our service and all services in the inherit tree. The services we inherit from are already implemented in other packages and we want use them. To do this we add this services to `iop_code_generator::EXTERN_SERVICES`. Thereby we use the directory name created by JTS for these services.
> ! internally our build script deletes all generated sources defined in `EXTERN_SERVICES` after creation by JTS.

Now we open a terminal and go to our package and run `catkin build --this`. This generates in `catkin_ws/build/iop_visual_sensor_fkie/jaus/Iop_visual_sensor_fkie_0` the source files for our service. In order to extend the functionality of the generated service, we copy the following files into our package, respecting the directory structure:
```
    include/urn_jaus_jss_environmentSensing_VisualSensor/VisualSensor_ReceiveFSM.h
    src/urn_jaus_jss_environmentSensing_VisualSensor/VisualSensor_ReceiveFSM.cpp
```
Additionally we create an empty file `src/main.cpp`.

These files we add to `iop_code_generator::OVERRIDES` in `CMakeLists.txt`:
```
    iop_code_generator(
      :
      OVERRIDES
        include/urn_jaus_jss_environmentSensing_VisualSensor/VisualSensor_ReceiveFSM.h
        src/urn_jaus_jss_environmentSensing_VisualSensor/VisualSensor_ReceiveFSM.cpp
        src/main.cpp
      :
    )
```

Now we can implement the functionality for the service by extending the two files.


### Add plugin functionality

Additionally we have to add the plugin functionality! We have to create a plugin class inherited from `iop::PluginInterface` which is localted at `iop_component_fkie/iop_plugin_interface.h`. The plugin class should create an instance of our new JAUS service by providing the needed parameter to it. We create two files `src/VisualSensorPlugin.h` and `src/VisualSensorPlugin.cpp` with folling content:
#### src/VisualSensorPlugin.h:
```cpp
#ifndef VISUALSENSORPLUGIN_H
#define VISUALSENSORPLUGIN_H

#include "urn_jaus_jss_environmentSensing_VisualSensor/VisualSensorService.h"
#include "urn_jaus_jss_core_AccessControl/AccessControlService.h"
#include "urn_jaus_jss_core_Events/EventsService.h"
#include "urn_jaus_jss_core_Transport/TransportService.h"

#include <iop_component_fkie/iop_plugin_interface.h>

namespace iop
{

class DllExport VisualSensorPlugin : public PluginInterface
{
public:
	VisualSensorPlugin();

	JTS::Service* get_service();
	void create_service(JTS::JausRouter* jaus_router);

protected:
	urn_jaus_jss_environmentSensing_VisualSensor::VisualSensorService *p_my_service;
	urn_jaus_jss_core_AccessControl::AccessControlService *p_base_service;
	urn_jaus_jss_core_Events::EventsService *p_events_service;
	urn_jaus_jss_core_Transport::TransportService *p_transport_service;

};

};

#endif
```

#### src/VisualSensorPlugin.cpp:
```cpp
#include <pluginlib/class_list_macros.h>
#include "VisualSensorPlugin.h"

using namespace iop;
using namespace urn_jaus_jss_environmentSensing_VisualSensor ;
using namespace urn_jaus_jss_core_AccessControl;
using namespace urn_jaus_jss_core_Events;
using namespace urn_jaus_jss_core_Transport;


VisualSensorPlugin::VisualSensorPlugin()
{
	p_my_service = NULL;
	p_base_service = NULL;
	p_events_service = NULL;
	p_transport_service = NULL;
}

JTS::Service* VisualSensorPlugin::get_service()
{
	return p_my_service;
}

void VisualSensorPlugin::create_service(JTS::JausRouter* jaus_router)
{
	p_base_service = static_cast<AccessControlService *>(get_base_service());
	p_events_service = static_cast<EventsService *>(get_base_service(2));
	p_transport_service = static_cast<TransportService *>(get_base_service(3));
	p_my_service = new VisualSensorService(jaus_router, p_transport_service, p_events_service, p_base_service);
}

PLUGINLIB_EXPORT_CLASS(iop::VisualSensorPlugin, iop::PluginInterface)
```

Do not forget to add the .cpp file to the `CMakeLists.txt`:
```makefile
    add_library(${PROJECT_NAME}
                src/VisualSensorPlugin.cpp
                ${cppfiles}
    )
```

Now we specify the IOP plugin description, so it can be found if we include it into our component. We create a new file `plugin_iop.xml` in root of our package with follow content:
```
    <library path="libiop_visual_sensor_fkie">
      <class name="VisualSensor" type="iop::VisualSensorPlugin" base_class_type="iop::PluginInterface">
        <description>
          VisualSensor Service Plugin
        </description>
        <iop_service name="VisualSensor" id="urn:jaus:jss:environmentSensing:VisualSensor" version="1.0">
          <inherits_from id="urn:jaus:jss:core:AccessControl" min_version="1.0"/>
        </iop_service>
      </class>
    </library>
```
The values for *iop_service* tag are taken from JSIDL definition. The *class* tag describes our plugin and the class we created. For more details see [Bridge-Plugins](component_package.md).

Then we include our plugin definition to the `package.xml`:
```
    <export>
      <iop_component_fkie plugin="${prefix}/plugin_iop.xml" />
    </export>
```

Now is the `package.xml` done.

### Provide catkin specific configuration `CMakeLists.txt`
Set include directories:
``` include_directories(${catkin_INCLUDE_DIRS})```

Add source files to the library:
```add_library(${PROJECT_NAME} src/VisualSensorPlugin.cpp ${cppfiles}) ```

Specify libraries to link a library against:
```target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})```

The service of this package is used by other packages, like *iop_digital_video_fkie* or *iop_still_image_fkie*. So that catkin works properly we have to configure `catkin_package`:
```makefile
    catkin_package(
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS iop_accesscontrol_fkie
    )
```

If the package will be used as installed package we have to add install directives:

```makefile
    # Mark IOP include files for installation
    install(
      DIRECTORY ${IOP_INSTALL_INCLUDE_DIRS} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
      PATTERN "*.old" EXCLUDE
      PATTERN "*.gen" EXCLUDE
    )

    # Mark executables and/or libraries for installation
    install(TARGETS ${PROJECT_NAME}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    ## Mark other files for installation (e.g. launch and bag files, etc.)
    install(
       FILES ./plugin_iop.xml
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
```

done! Build the package with `catkin build iop_visual_sensor_fkie`