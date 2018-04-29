### iop_builder_fkie
It contains the basic functionality and must be included in each component of the IOP/ROS bridge. The detailed overview of this package you find [here](../iop_builder_fkie/README.md).

### iop_component_fkie
Contains the plugin interface for all IOP services. The services which implement this interface can be included into the component on start. The **iop_component** binary represents an IOP component. The services of this component can be configured dynamically. For further details see [how it works](../doc/how_it_works.md) or [simulation example](https://github.com/fkie/iop_cfg_sim_stage_fkie/blob/master/README.md)

### iop_ocu_slavelib_fkie
A helper library for ROS/IOP-Bridge OCU nodes. The detailed overview of this package you find [here](../iop_ocu_slavelib_fkie/README.md).

## JAUS Core Services

[iop_accesscontrol_fkie: AccessControl](../iop_accesscontrol_fkie/README.md)  
[iop_accesscontrol_fkie: AccessControlClient](../iop_accesscontrol_fkie/README.md#iop_accesscontrol_fkie-accesscontrolclient)  
[iop_discovery_fkie: Discovery](../iop_discovery_fkie/README.md)  
[iop_discovery_fkie: DiscoveryClient](../iop_discovery_fkie/README.md#iop_discovery_fkie-discoveryclient)  
[iop_events_fkie: Events](../iop_events_fkie/README.md)  
[iop_events_fkie: EventsClient](../iop_events_fkie/README.md#iop_events_fkie-eventsclient)  
[iop_list_manager_fkie: ListManager](../iop_list_manager_fkie/README.md)  
[iop_list_manager_fkie: ListManagerClient](../iop_list_manager_fkie/README.md#iop_list_manager_fkie-listmanagerclient)  
[iop_liveness_fkie: Liveness](../iop_liveness_fkie/README.md)  
[iop_management_fkie: Management](../iop_management_fkie/README.md)  
[iop_management_fkie: ManagementClient](../iop_management_fkie/README.md#iop_management_fkie-managementclient)  
[iop_transport_fkie: Transport](../iop_transport_fkie/README.md)  
