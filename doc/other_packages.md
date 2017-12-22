## Main Repository of ROS/IOP Bridge
Core pakages of the ROS/IOP-Bridge needed to build the bridge. For interface description of core packages see [here](doc/iop_core_packages.md). The interface description of other services is located in corresponding repository.

_git clone https://github.com/fkie/iop_core_

## Repositories with packages implementing the SEA JAUS Services

- **core**: _urn.jaus.jss.core_ - services
    - _git clone https://github.com/fkie/iop_jaus_core_
    - iop_accesscontrol_fkie
    - iop_discovery_fkie
    - iop_events_fkie
    - iop_list_manager_fkie
    - iop_liveness_fkie
    - iop_management_fkie
    - iop_transport_fkie

- **manipulator**: _urn.jaus.jss.manipulator-v2.0_ - services
    - _git clone https://github.com/fkie/iop_jaus_manipulator
    - iop_manipulator_joint_position_sensor_fkie
    - iop_manipulator_specification_service_fkie
    - iop_pantilt_joint_position_driver_fkie
    - iop_pantilt_specification_service_fkie
    - iop_primitive_endeffector_fkie
    - iop_primitive_manipulator_fkie
    - iop_primitive_pantilt_fkie

- **mobility**: _urn.jaus.jss.mobility_ - services
    - _git clone https://github.com/fkie/iop_jaus_mobility_
    - iop_global_pose_sensor_fkie
    - iop_global_waypoint_driver_fkie
    - iop_global_waypoint_list_driver_fkie
    - iop_local_pose_sensor_fkie
    - iop_primitive_driver_fkie

- **sensing**: _urn.jaus.jss.environmentSensing_ - services
    - _git clone https://github.com/fkie/iop_jaus_sensing_
    - iop_digital_video_fkie
    - iop_range_sensor_fkie
    - iop_still_image_fkie
    - iop_visual_sensor_fkie

- **ugv**: _urn.jaus.jss.ugv_ - services
    - _git clone https://github.com/fkie/iop_jaus_ugv_
    - iop_stabilizer_driver_fkie


## Repositories with packages implementing the **clients** for SEA JAUS Services

The clients are used on the OCU side to control an IOP compliant robot.

- **manipulator_clients**: clients for _urn.jaus.jss.manipulator-v2.0_ - services
    - _git clone https://github.com/fkie/iop_jaus_manipulator_clients_
    - iop_client_manipulator_joint_position_sensor_fkie
    - iop_client_manipulator_specification_fkie
    - iop_client_pantilt_joint_position_driver_fkie
    - iop_client_pantilt_specification_service_fkie
    - iop_client_primitive_endeffector_fkie
    - iop_client_primitive_manipulator_fkie
    - iop_client_primitive_pantilt_fkie

- **mobility_clients**: clients for _urn.jaus.jss.mobility_ - services
    - _git clone https://github.com/fkie/iop_jaus_mobility_clients_
    - iop_client_global_pose_sensor_fkie
    - iop_client_global_waypoint_driver_fkie
    - iop_client_global_waypoint_list_driver_fkie
    - iop_client_local_pose_sensor_fkie
    - iop_client_primitive_driver_fkie

- **sensing_clients**: clients for _urn.jaus.jss.environmentSensing_ - services
    - _git clone https://github.com/fkie/iop_jaus_sensing_clients_
    - iop_client_digital_video_fkie
    - iop_client_range_sensor_fkie
    - iop_client_still_image_fkie
    - iop_client_visual_sensor_fkie

- **ugv**: clients for _urn.jaus.jss.ugv_ - services
    - _git clone https://github.com/fkie/iop_jaus_ugv_clients_
    - iop_client_stabilizer_driver_fkie

## IOP services
- **iop_platform**: the platform services including all services to create a platform
    - _git clone https://github.com/fkie/iop_platform_
    - iop_client_digital_resource_fkie
    - iop_digital_resource_discovery_fkie
    - iop_health_monitor_fkie
    - iop_platform_manager_fkie
    - iop_platform_mode_fkie
    - iop_platform_state_fkie

- **iop_sensing**: sensor services
    - _git clone https://github.com/fkie/iop_sensing_
    - iop_costmap2d_fkie
    - iop_measurement_sensor_fkie
    - iop_path_reporter_fkie

- **iop_sensing_clients**: clients for sensor services
    - _git clone https://github.com/fkie/iop_sensing_clients_
    - iop_client_costmap2d_fkie
    - iop_client_measurement_sensor_fkie
    - iop_client_path_reporter_fkie

## ROS GUI for IOP services
Basically we can use the available ROS GUIs. But some GUI components are very special, e.g. visualisation of the IOP components. In `gui` location are some GUI components using ROS *rqt*.
- _git clone https://github.com/fkie/iop_gui_
- iop_rqt_access_control_fkie
- iop_rqt_digital_resource_viewer_fkie

## Configuration Examples
- **iop_cfg_sim_stage_fkie**: Example with state simulator
	- _git clone https://github.com/fkie/iop_cfg_sim_stage_fkie_
