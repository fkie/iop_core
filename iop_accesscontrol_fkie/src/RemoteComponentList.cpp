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


#include <iop_accesscontrol_fkie/RemoteComponentList.h>
#include <algorithm>
#include <ros/ros.h>
#include <ros/console.h>

using namespace iop;

RemoteComponentList::RemoteComponentList(int default_timeout)
{
	p_default_timeout = default_timeout;
}

RemoteComponentList::~RemoteComponentList()
{
	p_components.clear();
}

bool RemoteComponentList::create(JausAddress address, jUnsignedByte authority)
{
	lock_type lock(p_mutex);
	if (!isin(address)) {
		boost::shared_ptr<iop::RemoteComponent> component(boost::make_shared<iop::RemoteComponent>(address, authority, p_default_timeout));
		p_components[address] = component;
		return true;
	}
	return false;
}

bool RemoteComponentList::remove(JausAddress address)
{
	lock_type lock(p_mutex);
	std::map<JausAddress, boost::shared_ptr<iop::RemoteComponent> >::iterator itcmp = p_components.find(address);
	if (itcmp != p_components.end()) {
		p_components.erase(itcmp);
		return true;
	}
	return false;
}

bool RemoteComponentList::isin(JausAddress address)
{
	lock_type lock(p_mutex);
	std::map<JausAddress, boost::shared_ptr<iop::RemoteComponent> >::iterator itcmp = p_components.find(address);
	return itcmp != p_components.end();
}

void RemoteComponentList::set_ack(JausAddress address, unsigned long secs)
{
	lock_type lock(p_mutex);
	std::map<JausAddress, boost::shared_ptr<iop::RemoteComponent> >::iterator itcmp = p_components.find(address);
	if (itcmp != p_components.end()) {
		itcmp->second->set_ack(secs);
	} else {
		ROS_DEBUG_NAMED("AccessControlClient", "can not set ack for %s, not found", address.str().c_str());
	}
}

void RemoteComponentList::set_insufficient_authority(JausAddress address)
{
	lock_type lock(p_mutex);
	std::map<JausAddress, boost::shared_ptr<iop::RemoteComponent> >::iterator itcmp = p_components.find(address);
	if (itcmp != p_components.end()) {
		itcmp->second->set_insufficient_authority();
	} else {
		ROS_DEBUG_NAMED("AccessControlClient", "can not set insufficient_authority for %s, not found", address.str().c_str());
	}
}

void RemoteComponentList::set_timeout(JausAddress address, int timeout)
{
	lock_type lock(p_mutex);
	std::map<JausAddress, boost::shared_ptr<iop::RemoteComponent> >::iterator itcmp = p_components.find(address);
	if (itcmp != p_components.end()) {
		itcmp->second->set_timeout(timeout);
	}
}

std::vector<boost::shared_ptr<iop::RemoteComponent> > RemoteComponentList::time_to_send_request(unsigned long deadtime)
{
	lock_type lock(p_mutex);
	std::vector<boost::shared_ptr<iop::RemoteComponent> > result;
	std::map<JausAddress, boost::shared_ptr<iop::RemoteComponent> >::iterator itcmp;
	for (itcmp = p_components.begin(); itcmp != p_components.end(); itcmp++) {
		if (itcmp->second->time_to_send_request(deadtime)) {
			result.push_back(itcmp->second);
		}
	}
	return result;
}

std::vector<boost::shared_ptr<iop::RemoteComponent> > RemoteComponentList::timeouted()
{
	lock_type lock(p_mutex);
	std::vector<boost::shared_ptr<iop::RemoteComponent> > result;
	std::map<JausAddress, boost::shared_ptr<iop::RemoteComponent> >::iterator itcmp;
	for (itcmp = p_components.begin(); itcmp != p_components.end(); itcmp++) {
		if (itcmp->second->timeouted()) {
			result.push_back(itcmp->second);
		}
	}
	return result;
}

bool RemoteComponentList::has_access(JausAddress address)
{
	lock_type lock(p_mutex);
	std::map<JausAddress, boost::shared_ptr<iop::RemoteComponent> >::iterator itcmp = p_components.find(address);
	if (itcmp != p_components.end()) {
		return itcmp->second->has_access();
	}
	return false;
}

