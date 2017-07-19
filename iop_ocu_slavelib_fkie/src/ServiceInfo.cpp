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

#include <boost/algorithm/string.hpp>
#include <iop_ocu_slavelib_fkie/common.h>
#include <iop_ocu_slavelib_fkie/ServiceInfo.h>

using namespace iop::ocu;


//ServiceInfo::ServiceInfo()
//{
//	p_major_version = 1;
//	p_minor_version = 0;
//	p_do_resume = false;
//	p_controled = false;
//	p_access_state = ServiceInfo::ACCESS_STATE_NOT_AVAILABLE;
//	p_access_control = ServiceInfo::ACCESS_CONTROL_RELEASE;
//	p_authority = 205;
//	p_info_updated = false;
//}

ServiceInfo::ServiceInfo(SlaveHandlerInterface &handler, JausAddress &own_addr, std::string uri, jUnsignedByte major_version, jUnsignedByte minor_version)
{
	p_handler = &handler;
	this->p_own_address = own_addr;
	this->p_uri = uri;
	this->p_major_version = major_version;
	this->p_minor_version = minor_version;
	p_do_resume = false;
	p_controled = false;
	p_access_state = ServiceInfo::ACCESS_STATE_NOT_AVAILABLE;
	p_access_control = ServiceInfo::ACCESS_CONTROL_RELEASE;
	p_authority = 205;
	p_info_updated = false;
}

ServiceInfo::~ServiceInfo(void)
{
}

SlaveHandlerInterface &ServiceInfo::handler()
{
	return *p_handler;
}

iop_msgs_fkie::OcuServiceInfo ServiceInfo::to_msg()
{
	iop_msgs_fkie::OcuServiceInfo result;
	result.addr_control = address_to_msg(p_address);
	result.uri = p_uri;
	result.access_state = p_access_state;
	result.authority = p_authority;
	p_info_updated = false;
	return result;
}

bool ServiceInfo::has_changes()
{
	return p_info_updated;
}

bool ServiceInfo::eqaulAddress(iop_msgs_fkie::JausAddress &a1, iop_msgs_fkie::JausAddress &a2)
{
	return ((a1.subsystem_id == a2.subsystem_id)
			&& (a1.node_id == a2.node_id)
			&& (a1.component_id == a2.component_id));
}

bool ServiceInfo::set_state(unsigned char access_state)
{
	bool changed = false;
	if (p_access_state != access_state) {
		changed = true;
		p_access_state = access_state;
	}
	p_info_updated = changed;
	return changed;
}

unsigned char ServiceInfo::get_state()
{
	return p_access_state;
}

unsigned char ServiceInfo::get_authority()
{
	return p_authority;
}

unsigned char ServiceInfo::get_access_control()
{
	return p_access_control;
}

JausAddress &ServiceInfo::get_own_address()
{
	return p_own_address;
}

JausAddress &ServiceInfo::get_address()
{
	return p_address;
}

std::string ServiceInfo::get_uri()
{
	return p_uri;
}

bool ServiceInfo::update_cmd(JausAddress &control_addr, unsigned char access_control, unsigned char authority)
{
	bool changed = false;
	JausAddress service_addr = pGetServiceAddress(control_addr);
	if (p_address.get() != service_addr.get()) {
		p_address = service_addr;
		changed = true;
	}
	if (p_access_control != access_control) {
		p_access_control = access_control;
		changed = true;
	}
	if (p_authority != authority) {
		p_authority = authority;
		changed = true;
	}
	p_info_updated = changed;
	return changed;
}

bool ServiceInfo::add_discovered(JausAddress address)
{
	std::pair<std::set<jUnsignedInteger>::iterator, bool> ret;
	ret = p_discovered_addresses.insert(address.get());
	return ret.second;
}

JausAddress ServiceInfo::get_component_addr(JausAddress &control_addr)
{
	if (control_addr.getComponentID() != 0) {
		return control_addr;
	}
	std::set<jUnsignedInteger>::iterator it;
	for (it = p_discovered_addresses.begin(); it != p_discovered_addresses.end(); ++it) {
		JausAddress da(*it);
		if (control_addr.getNodeID() == 0 || da.getNodeID() == control_addr.getNodeID()) {
			if (control_addr.getSubsystemID() == 0 || da.getSubsystemID() == control_addr.getSubsystemID()) {
				return da;
			}
		}
	}
	return JausAddress(0);
}

JausAddress ServiceInfo::pGetServiceAddress(JausAddress &control_addr)
{
	std::set<jUnsignedInteger>::iterator it;
	it = p_discovered_addresses.find(control_addr.get());
	if (it != p_discovered_addresses.end()) {
		return JausAddress(*it);
	}
	for (it=p_discovered_addresses.begin(); it!=p_discovered_addresses.end(); ++it) {
		JausAddress addr(*it);
		if (control_addr.getSubsystemID() == 0 || control_addr.getSubsystemID() == addr.getSubsystemID()) {
			if (control_addr.getNodeID() == 0 || control_addr.getNodeID() == addr.getNodeID()) {
				if (control_addr.getSubsystemID() == 0 || control_addr.getSubsystemID() == addr.getSubsystemID()) {
					return addr;
				}
			}
		}
	}
	return control_addr;
}

