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

#include <algorithm>    // std::find

#include <fkie_iop_ocu_slavelib/common.h>
#include <fkie_iop_ocu_slavelib/ServiceInfo.h>

using namespace iop::ocu;


ServiceInfo::ServiceInfo(SlaveHandlerInterface &handler, JausAddress &own_addr, std::string uri, jUnsignedByte major_version, jUnsignedByte minor_version)
{
	p_handler = &handler;
	this->p_own_address = own_addr;
	this->p_uri = uri;
	this->p_major_version = major_version;
	this->p_minor_version = minor_version;
}

ServiceInfo::~ServiceInfo(void)
{
}

SlaveHandlerInterface &ServiceInfo::handler()
{
	return *p_handler;
}

bool ServiceInfo::has_component(JausAddress &address)
{
	return std::find(p_discovered_addresses.begin(), p_discovered_addresses.end(), address) != p_discovered_addresses.end();
}

JausAddress &ServiceInfo::get_own_address()
{
	return p_own_address;
}

JausAddress &ServiceInfo::get_address()
{
	return p_address;
}

void ServiceInfo::set_address(JausAddress &address)
{
	p_address = address;
}

std::string ServiceInfo::get_uri()
{
	return p_uri;
}

bool ServiceInfo::add_discovered(JausAddress address, std::string service_uri, jUnsignedByte major_version, jUnsignedByte minor_version)
{
	if (!(major_version == p_major_version || major_version == 255 || p_major_version == 255)
		|| !(minor_version == p_minor_version || minor_version == 255 || p_minor_version == 255)
		|| p_uri.compare(service_uri) != 0) {
		return false;
	}
	if (!has_component(address)) {
		p_discovered_addresses.push_back(address);
		return true;
	}
	return false;
}

JausAddress ServiceInfo::get_dicovered_address(JausAddress filter, int nr)
{
	if (nr >= 1) {
		int idx = 1;
		std::vector<JausAddress>::iterator it_addr;
		for (it_addr = p_discovered_addresses.begin(); it_addr != p_discovered_addresses.end(); ++it_addr) {
			if (match_address(*it_addr, filter)) {
				if (idx == nr) {
					return *it_addr;
				}
				idx++;
			}
		}
	}
	return JausAddress(0);
}

