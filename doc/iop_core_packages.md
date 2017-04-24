## iop_builder_fkie
It contains the basic functionality and must be included in each component of the IOP/ROS bridge. The detailed overview of this package you find [here](builder_package.md).

## iop_msgs_fkie
Defines some ROS messages needed to represent the IOP structure, e.g. to visualize the current available IOP components.
>Please use for new components _standard_ ROS messages for translation of IOP messages if possible.

## iop_component_fkie
Contains the plugin interface for all IOP services. The services which implement this interface can be included into the component on start. The **iop_component** binary represents an IOP component. The services of this component can be configured dynamically. For further details see [how it works](how_it_works.md) or [simulation example](https://github.com/fkie/iop_cfg_sim_stage_fkie/blob/master/README.md)

## iop_ocu_controllib_fkie
The implementation of `OcuControlMaster` and `OcuControlSlave` interfaces. These are used by OCU-components to reduce the control effort while release or get access for many running components at the same time.

## iop_ocu_control_layerlib_fkie
Implements the communication between `OcuControlMaster` and `OcuControlSlave`. Reduces the effort of the implementation for OCU components.

