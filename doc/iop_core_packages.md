## iop_builder_fkie
It contains the basic functionality and must be included in each component of the IOP/ROS bridge. The detailed overview of this package you find [here](builder_package.md).

## iop_msgs_fkie
Defines some ROS messages needed to represent the IOP structure, e.g. to visualize the current available IOP components.
>Please use for new components _standard_ ROS messages for translation of IOP messages if possible.

## iop_component_fkie
Contains the plugin interface for all IOP services. The services which implement this interface can be included into the component on start. The **iop_component** binary represents an IOP component. The services of this component can be configured dynamically. For further details see [how it works](how_it_works.md) or [simulation example](https://github.com/fkie/iop_cfg_sim_stage_fkie/blob/master/README.md)

## iop_ocu_slavelib_fkie
A helper library for ROS/IOP-Bridge OCU nodes. This library combines the IOP client services (AccessControlClient, DiscoveryClient and ManagementClient) and listen to the command of the master (ocu_rqt_access_control_fkie) to synchronize the access of multiple OCU client to robot services.
Each OCU client needs to implement `SlaveHandlerInterface` interface class.

#### Parameter:

_control_addr (str_, (Default: "")

> The JAUS address if only a specific component, node or system are controlled by the client. Supported formats: XX, XX.XX or XX.XX.XX

_authority (int_, (Default: 205)

> The authority for access control.

_access_control (int_, (Default: 10)

> Access state after start the component. States: 10:ACCESS_CONTROL_RELEASE, 11:ACCESS_CONTROL_MONITOR, 12:ACCESS_CONTROL_REQUEST

_use_queries (bool_, (Default: false)

> Use queries on __true__ instead of events to get reports from robot services.

_only_monitor (bool_, (Default: false)

> Uses client only for monitoring if __true__ and does not allow access control.

_subsystem_restricted (int_, (Default: 65535)

> This client can control only the specified subsystem (robot). Any by default.


#### Publisher:

_/ocu_feedback (iop_msgs_fkie::OcuFeedback)_

> Publishes the current control state and occurred errors.

#### Subscriber:

_/ocu_cmd (iop_msgs_fkie::OcuCmd)_

> Receives the commands from iop_rqt_access_control GUI.
