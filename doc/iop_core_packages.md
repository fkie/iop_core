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

