## Main Repository of ROS/IOP Bridge
Core pakages of the ROS/IOP-Bridge needed to build the bridge. You find the description of containing ROS packages [here](doc/other_packages.md).

_git clone https://gihub.com/fkie/iop_core_

## Repositories with packages implementing the SEA JAUS Services

- **core**: _urn.jaus.jss.core_ - services
    - _git clone https://gihub.com/fkie/iop_jaus_core_
    - iop_accesscontrol_fkie
    - iop_discovery_fkie
    - iop_events_fkie
    - iop_liveness_fkie
    - iop_management_fkie
    - iop_transport_fkie

- **mobility**: _urn.jaus.jss.mobility_ - services
    - _git clone https://gihub.com/fkie/iop_jaus_mobility_
    - iop_global_pose_sensor_fkie
    - iop_local_pose_sensor_fkie
    - iop_primitive_driver_fkie

- **sensing**: _urn.jaus.jss.environmentSensing_ - services
    - _git clone https://gihub.com/fkie/iop_jaus_sensing_
    - iop_digital_video_fkie
    - iop_range_sensor_fkie


## Repositories with packages implementing the **clients** for SEA JAUS Services

The clients are used on the OCU side to control an IOP compliant robot.

- **mobility_clients**: clients for _urn.jaus.jss.mobility_ - services
    - _git clone https://gihub.com/fkie/iop_jaus_mobility_clients_
    - iop_client_global_pose_sensor_fkie
    - iop_client_local_pose_sensor_fkie
    - iop_client_primitive_driver_fkie

- **sensing_clients**: clients for _urn.jaus.jss.environmentSensing_ - services
    - _git clone https://gihub.com/fkie/iop_jaus_sensing_clients_
    - iop_client_range_sensor_fkie

## IOP services
- **iop_platform**: the platform services including all services to create a platform
    - _git clone https://gihub.com/fkie/iop_platform_
    - iop_digital_resource_discovery_fkie
    - iop_health_monitor_fkie
    - iop_platform_manager_fkie
    - iop_platform_mode_fkie
    - iop_platform_state_fkie

- **iop_sensing**: sensor services
    - _git clone https://gihub.com/fkie/iop_sensing_
    - iop_costmap2d_fkie

## ROS GUI for IOP services
Basically we can use the available ROS GUIs. But some GUI components are very special, e.g. visualisation of the IOP components. In `gui` location are some GUI components using ROS *rqt*.
- _git clone https://gihub.com/fkie/iop_gui_
- iop_rqt_access_control_fkie
- iop_rqt_digital_resource_viewer_fkie

## Configuration Examples
- **iop_cfg_sim_stage_fkie**: Example with state simulator
	- _git clone https://gihub.com/fkie/iop_cfg_sim_stage_fkie_
