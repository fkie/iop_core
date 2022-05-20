## fkie_iop_ocu_slavelib

A helper library for ROS/IOP-Bridge OCU nodes. This library combines the IOP client services (AccessControlClient, DiscoveryClient and ManagementClient) and listen to the command of the master (ocu_rqt_access_control_fkie) to synchronize the access of multiple OCU client to robot services.
Each OCU client needs to implement `SlaveHandlerInterface` interface class. All client plugins use the functionality of the same Slave instance.

This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).

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

_controlled_component (int_, (Default: 1)

> Use this parameter if more than one component with requested service is available on robot. Returns the service in the list, beginning with 1.

_force_monitor_on_start (bool_, (Default: false)

> Start monitoring the component declared in 'control_addr' immediately. No effect if 'control_addr' is not set!


#### Publisher:

_/ocu_feedback (fkie_iop_msgs::OcuFeedback)_

> Publishes the current control state and occurred errors.

#### Subscriber:

_/ocu_cmd (fkie_iop_msgs::OcuCmd)_

> Receives the commands from iop_rqt_access_control GUI.
