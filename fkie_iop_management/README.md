This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_management:_ Management

Implements rudimentary management functions.

#### Parameter:

> None

#### Publisher:

_is_emergency (std_msgs::msg::Bool)_, latched

> Publishes __true__ if the component changes into emergency state settled  by client.

_is_ready (std_msgs::msg::Bool)_, latched

> Publishes __true__ if the component changes in the ready state.

#### Subscriber:

> None

## _fkie_iop_management:_ ManagementClient

Currently only "Resume" implemented. This is used by Slave library. The possibility to change into other state are not supported.

#### Parameter:

_hz (float_ , Default: 0.0)

> Sets how often the reports are requested. If ```by_query``` is __true__ hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```by_query``` is __false__ an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.

_by_query (bool_, (Default: false)

> By default the current state will be requested by creating an event. By setting this variable to __true__ the state is requested by query.


#### Publisher:

_mgmt_emergency (std_msgs::msg::Bool)_, latched

> Publishes __true__ if the component is in the emergency state.

_mgmt_status (std_msgs::String)_, latched

> Publishes current state of the component. States are "INIT", "READY", "STANDBY", "SHUTDOWN", "FAILURE", "EMERGENCY" and "UNKNOWN".


#### Subscriber:

_cmd_mgmt_emergency (std_msgs::msg::Bool)_

> Command to set the controlled component into emergency state.

_cmd_mgmt_reset (std_msgs::msg::Bool)_

> Command to set the controlled component into ready state.


