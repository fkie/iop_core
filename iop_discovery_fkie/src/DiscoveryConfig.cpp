/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */


#include <iop_discovery_fkie/DiscoveryConfig.h>
#include <ros/ros.h>
#include <ros/console.h>

namespace discovery_config
{

DiscoveryConfig::DiscoveryConfig(int system_id, int system_type, std::string name_subsystem, std::string name_node, int id_subsystem, int id_node)
{
	this->system_id = system_id; // report as subsystem
	this->system_type = system_type; // report as vehicle
	this->name_subsystem = name_subsystem;
	this->name_node = name_node;
	this->id_subsystem = id_subsystem;
	this->id_node = id_node;
	this->register_own_services = true;
}

DiscoveryConfig::~DiscoveryConfig()
{
}

void DiscoveryConfig::update_ros_parameter()
{
	// Get the parameter for component/node/subsystem identification
	bool search_for_id_params = true;
//	int id_component = 0;
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	std::string iop_address;
	float iop_address_float;
	int iop_address_int;
	if (pnh.getParam("iop_address", iop_address)) {
		int p1, p2, p3;
		int scan_result = std::sscanf(iop_address.c_str(), "%d.%d.%d", &p1, &p2, &p3);
		if (scan_result == 2) {
			id_subsystem = p1;
			id_node = p2;
			search_for_id_params = false;
		} else if (scan_result == 3) {
			id_subsystem = p1;
			id_node = p2;
//			id_component = p3; // ignore the component ID
			search_for_id_params = false;
		} else {
			ROS_WARN("invalid format in ~iop_address[str]: %s, should be subsystem.node.component or subsystem.node", iop_address.c_str());
		}
	} else if (pnh.getParam("iop_address", iop_address_float)) {
		throw std::runtime_error("found ~iop_address, but with invalid type: float");
	} else if (pnh.getParam("iop_address", iop_address_int)) {
		throw std::runtime_error("found ~iop_address, but with invalid type: int");
	}
	// if no ~iop_address was found or it was invalid, we search for id_.. parameter
	ROS_DEBUG_ONCE_NAMED("DiscoveryConfig", "discovery parameter:");
	if (search_for_id_params) {
	if (pnh.getParam("id_subsystem", id_subsystem)) {
		ROS_DEBUG_ONCE_NAMED("DiscoveryConfig", "	found ~id_subsystem: %d", id_subsystem);
	} else if (nh.getParam("id_subsystem", id_subsystem)) {
		ROS_DEBUG_ONCE_NAMED("DiscoveryConfig", "	found id_subsystem: %d", id_subsystem);
	}
	if (pnh.getParam("id_node", id_node)) {
		ROS_DEBUG_ONCE_NAMED("DiscoveryConfig", "	found ~id_node: %d", id_node);
	} else if (nh.getParam("id_node", id_node)) {
		ROS_DEBUG_ONCE_NAMED("DiscoveryConfig", "	found id_node: %d", id_node);
	}
	}
	if (pnh.getParam("name_subsystem", name_subsystem)) {
		ROS_DEBUG_ONCE_NAMED("DiscoveryConfig", "	found ~name_subsystem: %s", name_subsystem.c_str());
	} else if (nh.getParam("name_subsystem", name_subsystem)) {
		ROS_DEBUG_ONCE_NAMED("DiscoveryConfig", "	found name_subsystem: %s", name_subsystem.c_str());
	}
	if (pnh.getParam("system_id", system_id)) {
		ROS_DEBUG_ONCE_NAMED("DiscoveryConfig", "	found ~system_id: %d", system_id);
	} else if (nh.getParam("system_id", system_id)) {
		ROS_DEBUG_ONCE_NAMED("DiscoveryConfig", "	found system_id: %d", system_id);
	}
	if (pnh.getParam("system_type", system_type)) {
		ROS_DEBUG_ONCE_NAMED("DiscoveryConfig", "	found ~system_type: %d", system_type);
	} else if (nh.getParam("system_type", system_type)) {
		ROS_DEBUG_ONCE_NAMED("DiscoveryConfig", "	found system_type: %d", system_type);
	}
	if (pnh.getParam("name_node", name_node)) {
		ROS_DEBUG_ONCE_NAMED("DiscoveryConfig", "	found ~name_node: %s", name_node.c_str());
	}
	if (pnh.getParam("register_own_services", register_own_services)) {
		ROS_DEBUG_ONCE_NAMED("DiscoveryConfig", "	found ~register_own_services: %d", register_own_services);
	}
	ROS_INFO_ONCE_NAMED("DiscoveryConfig", "used parameter by discovery:");
	ROS_INFO_ONCE_NAMED("DiscoveryConfig", "	system_id: %d (%s)", system_id, system_id_type2str(system_id).c_str());
	ROS_INFO_ONCE_NAMED("DiscoveryConfig", "	system_type: %d (%s)", system_type, system_type2str(system_type).c_str());
	ROS_INFO_ONCE_NAMED("DiscoveryConfig", "	name_subsystem: %s", name_subsystem.c_str());
	ROS_INFO_ONCE_NAMED("DiscoveryConfig", "	name_node: %s", name_node.c_str());
	ROS_INFO_ONCE_NAMED("DiscoveryConfig", "	id_subsystem: %d", id_subsystem);
	ROS_INFO_ONCE_NAMED("DiscoveryConfig", "	id_node: %d", id_node);
	ROS_INFO_ONCE_NAMED("DiscoveryConfig", "	register_own_services: %d", register_own_services);
}

std::string DiscoveryConfig::system_id_type2str(int system_id_type)
{
	switch (system_id_type) {
		case 1: return "System";
		case 2: return "Subsystem";
		case 3: return "Node";
		case 4: return "Component";
		default: return "Reserved";
	}
}

std::string DiscoveryConfig::system_type2str(int system_type)
{
	switch (system_type) {
		case 10001: return "VEHICLE";
		case 20001: return "OCU";
		case 30001: return "OTHER_SUBSYSTEM";
		case 40001: return "NODE";
		case 50001: return "PAYLOAD";
		case 60001: return "COMPONENT";
		default: return "Reserved";
	}
}

};
