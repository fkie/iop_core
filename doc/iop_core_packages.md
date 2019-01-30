### fkie_iop_builder
It contains the basic functionality and must be included in each component of the IOP/ROS bridge. The detailed overview of this package you find [here](../fkie_iop_builder/README.md).

### fkie_iop_component
Contains the plugin interface for all IOP services. The services which implement this interface can be included into the component on start. The **fkie_iop_component** binary represents an IOP component. The services of this component can be configured dynamically. For further details see [how it works](../doc/how_it_works.md) or [simulation example](https://github.com/fkie/iop_examples/blob/master/fkie_iop_cfg_sim_turtle/README.md)

### fkie_iop_ocu_slavelib
A helper library for ROS/IOP-Bridge OCU nodes. The detailed overview of this package you find [here](../fkie_iop_ocu_slavelib/README.md).

## JAUS Core Services

[fkie_iop_accesscontrol: AccessControl](../fkie_iop_accesscontrol/README.md)  
[fkie_iop_accesscontrol: AccessControlClient](../fkie_iop_accesscontrol/README.md#fkie_iop_accesscontrol-accesscontrolclient)  
[fkie_iop_discovery: Discovery](../fkie_iop_discovery/README.md)  
[fkie_iop_discovery: DiscoveryClient](../fkie_iop_discovery/README.md#fkie_iop_discovery-discoveryclient)  
[fkie_iop_events: Events](../fkie_iop_events/README.md)  
[fkie_iop_events: EventsClient](../fkie_iop_events/README.md#fkie_iop_events-eventsclient)  
[fkie_iop_list_manager: ListManager](../fkie_iop_list_manager/README.md)  
[fkie_iop_list_manager: ListManagerClient](../fkie_iop_list_manager/README.md#fkie_iop_list_manager-listmanagerclient)  
[fkie_iop_liveness: Liveness](../fkie_iop_liveness/README.md)  
[fkie_iop_management: Management](../fkie_iop_management/README.md)  
[fkie_iop_management: ManagementClient](../fkie_iop_management/README.md#fkie_iop_management-managementclient)  
[fkie_iop_transport: Transport](../fkie_iop_transport/README.md)  
