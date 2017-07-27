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

#include <boost/algorithm/string.hpp>
#include <iop_ocu_slavelib_fkie/common.h>
#include <iop_ocu_slavelib_fkie/ServiceInfo.h>

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
	return p_discovered_addresses.find(address.get()) != p_discovered_addresses.end();
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

bool ServiceInfo::add_discovered(JausAddress address)
{
	std::pair<std::set<jUnsignedInteger>::iterator, bool> ret;
	ret = p_discovered_addresses.insert(address.get());
	return ret.second;
}

