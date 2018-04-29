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

#include <iop_discovery_fkie/DiscoveryComponent.h>

using namespace iop;


DiscoveryComponent::DiscoveryComponent() {
	ts_last_ident = 0;
}

DiscoveryComponent::DiscoveryComponent(JausAddress address) {
	this->address = address;
	ts_last_ident = 0;
}

bool DiscoveryComponent::add_service(std::string service_uri, unsigned char major_version, unsigned char minor_version)
{
	return p_services.add_service(service_uri, major_version, minor_version);
}

std::vector<DiscoveryServiceDef> DiscoveryComponent::get_services()
{
	return p_services.get_services();
}

bool DiscoveryComponent::has_service(std::string uri)
{
	return p_services.has_service(uri);
}

// comparable for the map
bool DiscoveryComponent::operator<( const DiscoveryComponent& other) const
{
	return (address.get() < other.address.get());
}

bool DiscoveryComponent::operator==( const DiscoveryComponent& other) const
{
	return (address == other.address);
}

bool DiscoveryComponent::operator==( const JausAddress& other) const
{
	return (address == other);
}
