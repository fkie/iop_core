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


#ifndef DISCOVERY_COMPONENT_LIST_H
#define DISCOVERY_COMPONENT_LIST_H

#include <rclcpp/rclcpp.hpp>
#include "Transport/JausTransport.h"
#include "DiscoveryComponent.h"

namespace iop
{

class DiscoveryComponentList {
public:
	/**
	 * :param timeout: after this timeout a component will be removed on access or update. Zero disables timeout. */
	DiscoveryComponentList(rclcpp::Logger logger, int64_t timeout=60);
	void set_timeout(int64_t timeout);

	bool add_service(JausAddress discovery_service, JausAddress component, std::string service_uri, unsigned char major_version, unsigned char minor_version=255);
	/** Returns false if component not in the list or was expired and removed since last update. */
	bool update_ts(JausAddress discovery_service, JausAddress component);
	bool update_ts(JausAddress discovery_service, unsigned short subsystem, unsigned char node=255);
	std::vector<DiscoveryComponent> get_components(JausAddress discovery_service, unsigned short subsystem=65535, unsigned char node=255, unsigned char component=255);
	std::vector<DiscoveryComponent> get_components(JausAddress discovery_service, std::string uri);
	void remove_discovery_service(JausAddress addr);
	std::vector<JausAddress> get_discovery_services();

protected:
	rclcpp::Logger logger;
	std::map<JausAddress, std::vector<DiscoveryComponent> > p_components;
	int64_t p_timeout;

	bool p_expired(int64_t ts, int64_t now=0);
	std::vector<DiscoveryComponent>& p_get_components(JausAddress discovery_service);
};

}

#endif // DISCOVERY_CONFIG_H
