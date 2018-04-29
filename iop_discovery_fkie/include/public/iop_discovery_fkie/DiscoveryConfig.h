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


#ifndef DISCOVERY_CONFIG_H
#define DISCOVERY_CONFIG_H

#include <string>

namespace discovery_config
{

const int TYPE_SYSTEM = 1;
const int TYPE_SUBSYSTEM = 2;
const int TYPE_NODE = 3;
const int TYPE_COMPONENT = 4;

class DiscoveryConfig
{
 public:
	DiscoveryConfig(int system_id=4, int system_type=60001, std::string name_subsystem="Robotname", std::string name_node="Payloadname", int id_subsystem=127, int id_node= 0x40);
	virtual ~DiscoveryConfig();
	std::string system_id_type2str(int system_id_type);
	std::string system_type2str(int system_type);

	/**
		0: Reserved
		1: System Identification
		2: Subsystem Identification
		3: Node Identification
		4: Component Identification
		5 – 255: Reserved
	 */
	int system_id;
	/**
		00000: Reserved
		10001: VEHICLE
		10002-20000: Reserved
		20001: OCU
		20002-30000: Reserved
		30001: OTHER_SUBSYSTEM
		30002-40000: Reserved
		40001: NODE
		40002-50000: Reserved
		50001: PAYLOAD
		50002-60000: Reserved
		60001: COMPONENT
		60002-65535: Reserved"
	**/
	int system_type;
	std::string name_subsystem;
	std::string name_node;
	int id_subsystem;
	int id_node;
	bool register_own_services;

	void update_ros_parameter();

};

};

#endif // DISCOVERY_CONFIG_H
