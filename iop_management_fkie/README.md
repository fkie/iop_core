This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _iop_management_fkie:_ Management

Implements rudimentary management functions.

#### Parameter:

> None

#### Publisher:

_is_emergency (std_msgs::Bool)_, latched

> Publishes __true__ if the component changes into emergency state settled  by client.

_is_ready (std_msgs::Bool)_, latched

> Publishes __true__ if the component changes in the ready state.

#### Subscriber:

> None

## _iop_management_fkie:_ ManagementClient

Currently only "Resume" implemented. This is used by Slave library. The possibility to change into other state are not supported.

#### Parameter:

> None

#### Publisher:

> None

#### Subscriber:

> None
