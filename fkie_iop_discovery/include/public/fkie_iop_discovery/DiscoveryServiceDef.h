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


#ifndef DISCOVERY_SERVICE_DEF_H
#define DISCOVERY_SERVICE_DEF_H

#include "Transport/JausTransport.h"
#include <string>

namespace iop
{

class DiscoveryServiceDef {
public:
	std::string service_uri;
	unsigned char major_version;
	unsigned char minor_version;

	DiscoveryServiceDef();
	DiscoveryServiceDef(std::string service_uri, unsigned char major_version, unsigned char minor_version);
	// comparable for the map
	bool operator<( const DiscoveryServiceDef& other) const;
	bool operator==( const DiscoveryServiceDef& other) const;
	bool operator!=( const DiscoveryServiceDef& other) const;
};

};


#endif // DISCOVERY_CONFIG_H
