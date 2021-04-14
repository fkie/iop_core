This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_accesscontrol:_ AccessControl

Implements the functionality for exclusive control. By default a timeout of 10 seconds is active. You have to send your requests within this time as not to lose the control.

#### Parameter:

_access_timeout (int_, (Default: 60)

> Time period in seconds after which the exclusive control goes lost. Zero disables the timeout.

_default_authority (int_, (Default: 1)

> The authority level of a client requesting control must be greather than or equal to the this value.

#### Publisher:

_is_controlled (std_msgs::Bool)_, latched

> Publishes True or False if the service is controlled by someone or not

_is_control_available (std_msgs::Bool)_, latched

> Publishes the state of control allowed from ROS side.

#### Subscriber:

_set_control_available (std_msgs::Bool)_

> If you set to False, all access control requests will be rejected.

## _fkie_iop_accesscontrol:_ AccessControlClient

The client service for AccessControl. It offers the functionality to send access requests in periodic time specified by AccessControl service.
