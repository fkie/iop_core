## Component Package - iop_component_fkie

Each IOP/JAUS service in ROS/IOP-Brdige is build as a plugin. The plugin interface is defined in the `iop_component_fkie` package. This package also offers a binary which loads all the plugins to a component. All components of the bridge are started unsing this package. To create a plugin for this component you must do two things:

1. Configure the plugin in `plugin_iop.xml` of your package.
2. Create a subclass for `iop::PluginInterface` from `iop_component_fkie/PluginInterface.h`.


### 1. Configuration in plugin_iop.xml

The plugin definition uses [ROS pluginlib](http://wiki.ros.org/pluginlib). The `class` definition is specified by `pluginlib`. As `base_class_type` you should use `iop::PluginInterface` and for `type` your plugin class defined in (2.). For the IOP specifications we added a new tag `iop_service`. This describes the service defined by the plugin. You have to set three arguments: `name`, `id` and `version`.
```
    <class ...>
        <iop_service name="Events" id="urn:jaus:jss:core:Events" version="1.1"/>
    </class>
```
#### name
The name of the managed IOP/JAUS service. It is also used while component configuration.

#### id
The JAUS id specified in JSIDL file.

#### version
The JAUS service version specified in JSIDL file.


The `iop_service` can contain two further tags: `inherits_from` and `depend`. `inherits_from` defines the base service, e.g.:
```
    <iop_service name="Events" id="urn:jaus:jss:core:Events" version="1.1">
        <inherits_from id="urn:jaus:jss:core:Transport" min_version="1.1"/>
    </iop_service>
```

The `depend` tag specifies services which should be included into the component if this plugin should work properly. For example the PrimitivePantilt service inherts from Management, but it needs also the PanTiltSpecificationService:
```
    <iop_service name="PrimitivePanTilt" id="urn:jaus:jss:manipulator:PrimitivePanTilt" version="2.0">
        <inherits_from id="urn:jaus:jss:core:Management" min_version="1.1"/>
        <depend id="urn:jaus:jss:manipulator:PanTiltSpecificationService"/>
    </iop_service>
```

### 2. Create a subclass for `iop::PluginInterface`

You have to subclass the `iop::PluginInterface` and overwrite at least two methods `create_service()` and `get_service()`. The `create_service` method is used to create the JAUS service we create by our plugin. See an example in [howto create plugin]([https://github.com/fkie/iop_core/blob/master/doc/howto_create_plugin.md#add-plugin-functionality)
