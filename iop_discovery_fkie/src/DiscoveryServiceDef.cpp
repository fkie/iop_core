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

#include <iop_discovery_fkie/DiscoveryServiceDef.h>

using namespace iop;

DiscoveryServiceDef::DiscoveryServiceDef()
{
	this->service_uri = "";
	this->major_version = 0;
	this->minor_version = 0;
}

DiscoveryServiceDef::DiscoveryServiceDef(std::string service_uri, unsigned char major_version, unsigned char minor_version)
{
	this->service_uri = service_uri;
	this->major_version = major_version;
	this->minor_version = minor_version;
}


// comparable for the map
bool DiscoveryServiceDef::operator<( const DiscoveryServiceDef& other) const
{
	bool mj_equal = (major_version == other.major_version);
	bool mn_equal = (minor_version == other.minor_version);
	return ((major_version < other.major_version) ||
			(mj_equal && (minor_version < other.minor_version)) ||
			(mj_equal && mn_equal && (service_uri < other.service_uri)));
}

bool DiscoveryServiceDef::operator==( const DiscoveryServiceDef& other) const
{
	bool mj_equal = (major_version == other.major_version || major_version == 255 || other.major_version == 255);
	bool mn_equal = (minor_version == other.minor_version || minor_version == 255 || other.minor_version == 255);
	return (mj_equal && mn_equal && (service_uri == other.service_uri));
}

bool DiscoveryServiceDef::operator!=( const DiscoveryServiceDef& other) const
{
	return !(*this == other);
}

