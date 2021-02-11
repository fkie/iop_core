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

#ifndef OCU_COMMON_H_
#define OCU_COMMON_H_

#include <Transport/JausAddress.h>
#include <fkie_iop_msgs/msg/jaus_address.hpp>

namespace iop
{

namespace ocu
{
	inline fkie_iop_msgs::msg::JausAddress address_to_msg(JausAddress addr) {
		fkie_iop_msgs::msg::JausAddress result;
		result.subsystem_id = addr.getSubsystemID();
		result.node_id = addr.getNodeID();
		result.component_id = addr.getComponentID();
		return result;
	}

	inline JausAddress address_from_msg(fkie_iop_msgs::msg::JausAddress addr) {
		JausAddress result;
		result.setSubsystemID(addr.subsystem_id);
		result.setNodeID(addr.node_id);
		result.setComponentID(addr.component_id);
		return result;
	}

	inline bool match_address(JausAddress &address, JausAddress &expr) {
		bool result = false;
		if (expr.getSubsystemID() != 0 && expr.getSubsystemID() != 65535) {
			if (expr.getSubsystemID() == address.getSubsystemID()) {
				if (expr.getNodeID() != 0 && expr.getNodeID() != 255) {
					if (expr.getNodeID() == address.getNodeID()) {
						if (expr.getComponentID() != 0 && expr.getComponentID() != 255) {
							if (expr.getComponentID() == address.getComponentID()) {
								result = true;
							}
						} else {
							result = true;
						}
					}
				} else {
					result = true;
				}
			}
		} else if (expr.getSubsystemID() == 65535) {
			result = true;
		}
		return result;
	}
}

}

#endif
