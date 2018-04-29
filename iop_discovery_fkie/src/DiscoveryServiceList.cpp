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

#include <iop_discovery_fkie/DiscoveryServiceList.h>
#include <algorithm>
#include <iostream>


using namespace iop;

DiscoveryServiceList::DiscoveryServiceList() {
}

bool DiscoveryServiceList::add_service(std::string service_uri, unsigned char major_version, unsigned char minor_version)
{
	bool added = false;
	DiscoveryServiceDef sdef(service_uri, major_version, minor_version);
	std::vector<DiscoveryServiceDef>::iterator it = std::find(p_services.begin(), p_services.end(), sdef);
	if (it == p_services.end()) {
		p_services.push_back(sdef);
		added = true;
	}
	return added;
}

std::vector<DiscoveryServiceDef> DiscoveryServiceList::get_services()
{
	return p_services;
}

bool DiscoveryServiceList::has_service(std::string uri)
{
	std::vector<DiscoveryServiceDef>::iterator it;
	for (it = p_services.begin(); it != p_services.end(); it++) {
		if (it->service_uri.compare(uri) == 0) {
			return true;
		}
	}
	return false;
}
