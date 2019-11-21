## How to start with own configuration

First of all you need a running `JAUS Node Manager` on **each host** where you start ROS/IOP components. `JAUS Node Manager` is part of JTS, but we created a script, so you can include it into ROS launch files, see [jaus_node_manager.launch](https://github.com/fkie/iop_examples/blob/master/fkie_iop_cfg_sim_stage/launch/jaus_node_manager.launch). Or you run it directly `JTSNodeManager path/to/nm.cfg`.

### Create configuration for robot

In JAUS/IOP a robot represents a subsystem with all his payloads and computers. For a whole subsystem we have to setup exact one component, which manage the discovering of all IOP/JAUS services. Usually it is a component named `platfrom_manager` with `node id` **15**.
```
    <param name="name_subsystem" value="Bob"/>
    <node name="iop_platform_manager" pkg="fkie_iop_component" type="iop_component">
        <param name="name_node" value="platform manager"/>
        <param name="iop_address" value="1.1.15"/>
    </node>
```
>IOPv2 defines dynamic address assignment. This feature currently not implemented in ROS/IOP Bridge. You have to define your own JAUS ID with Subsystem.Node.Component.

>`name_subsystem` parameter specifies the name of the subsystem discovered by OCU.

The minimal set of services in `platform_manager` fontains of:

* urn:jaus:jss:core:Discovery
* urn:jaus:jss:core:Liveness
* urn:jaus:jss:iop:DigitalResourceDiscovery
* urn:jaus:jss:exp:aeodrs:HealthMonitor

Add these services and also all serviceses inherits of to `platform_manager`:
```
    <param name="name_subsystem" value="Bob"/>
    <node name="iop_platform_manager" pkg="fkie_iop_component" type="iop_component">
        <param name="name_node" value="platform manager"/>
        <param name="iop_address" value="1.1.15"/>
        <rosparam param="services">
            [
            fkie_iop_transport: "Transport",
            fkie_iop_events: "Events",
            fkie_iop_events: "EventsClient",
            fkie_iop_accesscontrol: "AccessControl",
            fkie_iop_discovery: "Discovery",
            fkie_iop_discovery: "DiscoveryClient",
            fkie_iop_liveness: "Liveness",
            fkie_iop_digital_resource_discovery: "DigitalResourceDiscovery",
            fkie_iop_health_monitor: "HealthMonitor",
            ]
        </rosparam>
    </node>
```
`DiscoveryClient` service is included to register our own services by `Discovery` services, so they can be found by other components. `EventsClient` service is included because `DiscoveryClient` inherits of it.

Now we have to configure the `Discovery` service to enable the service registration management:
```
    <param name="name_subsystem" value="Bob"/>
    <node name="iop_platform_manager" pkg="fkie_iop_component" type="iop_component">
        <param name="name_node" value="platform manager"/>
        <param name="iop_address" value="1.1.15"/>
        <rosparam param="services">
            [
            fkie_iop_transport: "Transport",
            fkie_iop_events: "Events",
            fkie_iop_events: "EventsClient",
            fkie_iop_accesscontrol: "AccessControl",
            fkie_iop_discovery: "Discovery",
            fkie_iop_discovery: "DiscoveryClient",
            fkie_iop_liveness: "Liveness",
            fkie_iop_digital_resource_discovery: "DigitalResourceDiscovery",
            fkie_iop_health_monitor: "HealthMonitor",
            ]
        </rosparam>
        <rosparam subst_value="true">
            Discovery:
                # 2: Subsystem Identification, 3: Node Identification, 4: Component Identification
                system_id: 2
                # 10001: VEHICLE, 20001: OCU, 30001: OTHER_SUBSYSTEM, 40001: NODE, 50001: PAYLOAD, 60001: COMPONENT
                system_type: 10001
        </rosparam>
    </node>
```

This is the minimal configuration. The robot will be discovered by IOP OCU but will not provide any functional services. You can add this services to the `platform_manager` component or create a new one. See `iop_costmap2d` component in [fkie_iop_cfg_sim_stage](https://github.com/fkie/iop_examples/blob/master/fkie_iop_cfg_sim_stage/launch/inc_iop_robot.launch) example.

### Create configuration for OCU

Now we create a minimal configuration for a component running on OCU. You have to also specify the JAUS ID and can define a name:
```
    <node name="iop_ocu_client" pkg="fkie_iop_component" type="iop_component">
        <param name="iop_address" value="150.64.200"/>
        <param name="name_node" value="control_client"/>
        <rosparam param="services">
          [
            fkie_iop_transport: "Transport",
            fkie_iop_events: "EventsClient",
            fkie_iop_discovery: "DiscoveryClient",
          ]
        </rosparam>
        <rosparam subst_value="true">
            use_queries: false
            DiscoveryClient:
                register_own_services: false
                enable_ros_interface: true
        </rosparam>
    </node>
```

To discover the IOP robot you have to include the `DiscoveryClient`:
```
    <node name="iop_ocu_client" pkg="fkie_iop_component" type="iop_component">
        <param name="iop_address" value="150.64.200"/>
        <param name="name_node" value="control_client"/>
        <rosparam param="services">
          [
            fkie_iop_transport: "Transport",
            fkie_iop_events: "EventsClient",
            fkie_iop_discovery: "DiscoveryClient",
          ]
        </rosparam>
    </node>
```
Since the `DsicoveryClient` also used to register own services by `Discovery` services, the ROS interface for discovered services is disabled by default. You have to enable this by setting the parameter `enable_ros_interface` to `true`. Furthermore you can disable the registration of client service of the OCU, since they consumes only the functionality of robot service. And there is no need to register them in the subsystem:
```
    <node name="iop_ocu_client" pkg="fkie_iop_component" type="iop_component">
        <param name="iop_address" value="150.64.200"/>
        <param name="name_node" value="control_client"/>
        <rosparam param="services">
          [
            fkie_iop_transport: "Transport",
            fkie_iop_events: "EventsClient",
            fkie_iop_discovery: "DiscoveryClient",
          ]
        </rosparam>
        <rosparam subst_value="true">
            DiscoveryClient:
                register_own_services: false
                enable_ros_interface: true
        </rosparam>
    </node>
```

If you now launch JAUS Node Manager and new launch files you get the discovered system overview on OCU by calling
```
rostopic echo /iop_system
```

As with the robot, you can extend the functionality of the OCU client component by adding the services to the `services` parameter.
>Note: for services, which control the robot, you need to get **access control** (for managed services also resume from standby). For these porposes it is easier to use the *rqt* plugin from **fkie_iop_rqt_access_control**.

