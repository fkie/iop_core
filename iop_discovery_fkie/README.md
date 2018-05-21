This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _iop_discovery_fkie:_ Discovery

The discovery service holds information about System, Subsystem, Node or Component depend on the configuration parameter _system_id_. It should by only one service with type __Subsystem__ in your subsystem. Each component can include this service to provide a name of the component.

#### Parameter:

_system_id (int_, (Default: 4)

> The ID of the service: 0: Reserved, 1: System Identification, 2: Subsystem Identification, 3: Node Identification, 4: Component Identification, 5 – 255: Reserved

_system_type (int_, (Default: 60001)

> 10001: VEHICLE, 20001: OCU, 30001: OTHER_SUBSYSTEM, 40001: NODE, 50001: PAYLOAD, 60001: COMPONENT

_name_subsystem (str_, (Default: Robotname)

> The name of the robot. Only used if the _system_id_ is set to 2 (subsystem).

_name_node (str_, (Default: Componentname)

> The name of the component which includes this service.

_query_timeout_discover (int_, (Default: 5)

> Timeout to send a new query identification message to discover service. After _query_timeout_standby/query_timeout_discover_ count of queries the timeout is increased to query_timeout_standby.

_query_timeout_standby (int_, (Default: 15)

> Timeout to send a new query identification message to update already discovered service/components.

#### Publisher:

> None

#### Subscriber:

> None


## _iop_discovery_fkie:_ DiscoveryClient

A client service to discover other IOP services and register own services by Discovery service of _subsystem_ type. It provide also a ROS publisher with all discovered services.

#### Parameter:

_system_id (int_, (Default: 4)

> This parameter influence the discover update behaviour only if ROS interface is disabled and all IOP services are already discovered. The same type as in Discovery.

_register_own_services (bool_, (Default: true)

> You can prevent the registration of own services by discovery service. It is useful in case of OCU services.

_force_component_update_after (int_, (Default: 300)

> Force the request of service list for all known robot.

_enable_ros_interface (bool_, (Default: false)

> Publish the discovered services to the ROS network. On false the publisher are not created.

_unicast_subsystems (list_, (Default: [])

> A list of JAUS address (e.g. 123.45.67) of platform components with discovery service. It should be filled if your multicast communication does not work. It allows to discover this robot througth unicast communication. You need also to fill the `AddressBook` in jaus configuration file (usually nm.cfg in jaustoolset).
> This functionality is currently experimental and can be extended in future.

#### Publisher:

_/iop_identification (iop_msgs_fkie::Identification)_

> Publishes the identification report to the ROS network.

_/iop_system (iop_msgs_fkie::System)_, latched

> Publishes all services of all discovered subsystems on every detected change.

#### Subscriber:

> None

#### Services:

_/iop_query_identification (iop_msgs_fkie::QueryIdentification)_

> Request an identification for provided system id.

_/iop_update_discovery (std_srvs::Empty)_

> Tries to update the whole system.

