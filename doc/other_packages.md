## Main Repository of ROS/IOP Bridge
Core pakages of the ROS/IOP-Bridge needed to build the bridge. For interface description of core packages see [here](iop_core_packages.md). The interface description of other services is located in corresponding repository.

_git clone https://github.com/fkie/iop_core_

## Repositories with packages implementing the SEA JAUS Services

- **core**: _urn.jaus.jss.core_ - services
    - fkie_iop_accesscontrol
    - fkie_iop_discovery
    - fkie_iop_events
    - fkie_iop_list_manager
    - fkie_iop_liveness
    - fkie_iop_management
    - fkie_iop_transport

- **manipulator**: _urn.jaus.jss.manipulator-v2.0_ - services
    - _git clone [https://github.com/fkie/iop_jaus_manipulator](https://github.com/fkie/iop_jaus_manipulator)_
    - fkie_iop_manipulator_joint_position_sensor
    - fkie_iop_manipulator_specification_service
    - fkie_iop_pantilt_joint_position_driver
    - fkie_iop_pantilt_specification_service
    - fkie_iop_primitive_endeffector
    - fkie_iop_primitive_manipulator
    - fkie_iop_primitive_pantilt

- **mobility**: _urn.jaus.jss.mobility_ - services
    - _git clone [https://github.com/fkie/iop_jaus_mobility](https://github.com/fkie/iop_jaus_mobility)_
    - fkie_iop_global_pose_sensor
    - fkie_iop_global_waypoint_driver
    - fkie_iop_global_waypoint_list_driver
    - fkie_iop_local_pose_sensor
    - fkie_iop_local_waypoint_driver
    - fkie_iop_local_waypoint_list_driver
    - fkie_iop_primitive_driver
    - fkie_iop_velocity_state_sensor

- **sensing**: _urn.jaus.jss.environmentSensing_ - services
    - _git clone [https://github.com/fkie/iop_jaus_sensing](https://github.com/fkie/iop_jaus_sensing)_
    - fkie_iop_digital_video
    - fkie_iop_range_sensor
    - fkie_iop_still_image
    - fkie_iop_visual_sensor

- **ugv**: _urn.jaus.jss.ugv_ - services
    - _git clone [https://github.com/fkie/iop_jaus_ugv](https://github.com/fkie/iop_jaus_ugv)_
    - fkie_iop_illumination
    - fkie_iop_power_plant
    - fkie_iop_stabilizer_driver


## Repositories with packages implementing the **clients** for SEA JAUS Services

The clients are used on the OCU side to control an IOP compliant robot.

- **manipulator_clients**: clients for _urn.jaus.jss.manipulator-v2.0_ - services
    - _git clone [https://github.com/fkie/iop_jaus_manipulator_clients](https://github.com/fkie/iop_jaus_manipulator_clients)_
    - fkie_iop_client_manipulator_joint_position_sensor
    - fkie_iop_client_manipulator_specification
    - fkie_iop_client_pantilt_joint_position_driver
    - fkie_iop_client_pantilt_specification_service
    - fkie_iop_client_primitive_endeffector
    - fkie_iop_client_primitive_manipulator
    - fkie_iop_client_primitive_pantilt

- **mobility_clients**: clients for _urn.jaus.jss.mobility_ - services
    - _git clone [https://github.com/fkie/iop_jaus_mobility_clients](https://github.com/fkie/iop_jaus_mobility_clients)_
    - fkie_iop_client_global_pose_sensor
    - fkie_iop_client_global_waypoint_driver
    - fkie_iop_client_global_waypoint_list_driver
    - fkie_iop_client_local_pose_sensor
    - fkie_iop_client_local_waypoint_driver
    - fkie_iop_client_local_waypoint_list_driver
    - fkie_iop_client_primitive_driver
    - fkie_iop_client_velocity_state_sensor

- **sensing_clients**: clients for _urn.jaus.jss.environmentSensing_ - services
    - _git clone [https://github.com/fkie/iop_jaus_sensing_clients](https://github.com/fkie/iop_jaus_sensing_clients)_
    - fkie_iop_client_digital_video
    - fkie_iop_client_range_sensor
    - fkie_iop_client_still_image
    - fkie_iop_client_visual_sensor

- **ugv**: clients for _urn.jaus.jss.ugv_ - services
    - _git clone [https://github.com/fkie/iop_jaus_ugv_clients](https://github.com/fkie/iop_jaus_ugv_clients)_
    - fkie_iop_client_illumination
    - fkie_iop_client_power_plant
    - fkie_iop_client_stabilizer_driver

## IOP services
- **iop_platform**: the platform services including all services to create a platform
    - _git clone [https://github.com/fkie/iop_platform](https://github.com/fkie/iop_platform)_
    - fkie_iop_client_digital_resource
    - fkie_iop_digital_resource_discovery
    - fkie_iop_handoff
    - fkie_iop_health_monitor
    - fkie_iop_platform_manager
    - fkie_iop_platform_mode
    - fkie_iop_platform_state
    - fkie_iop_unsolicited_heartbeat

- **iop_sensing**: sensor services
    - _git clone [https://github.com/fkie/iop_sensing](https://github.com/fkie/iop_sensing)_
    - fkie_iop_costmap2d
    - fkie_iop_measurement_sensor
    - fkie_iop_path_reporter

- **iop_sensing_clients**: clients for sensor services
    - _git clone [https://github.com/fkie/iop_sensing_clients](https://github.com/fkie/iop_sensing_clients)_
    - fkie_iop_client_costmap2d
    - fkie_iop_client_measurement_sensor
    - fkie_iop_client_path_reporter


## ROS Messages
For some functionallity we defined ROS messages.

_git clone [https://github.com/fkie/iop_msgs](https://github.com/fkie/iop_msgs)_


## ROS GUI for IOP services
Basically we can use the available ROS GUIs. But some GUI components are very special, e.g. visualisation of the IOP components. In `gui` location are some GUI components using ROS *rqt*.

- _git clone [https://github.com/fkie/iop_gui](https://github.com/fkie/iop_gui)_
- fkie_iop_rqt_access_control
- fkie_iop_rqt_digital_resource_viewer

## Configuration Examples
- **iop_examples**: Example with state simulator
	- _git clone [https://github.com/fkie/iop_examples](https://github.com/fkie/iop_examples)_
	- iop_cfg_sim_stage
	- iop_cfg_sim_turtle
