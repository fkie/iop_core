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

#include <algorithm>
#include <ros/ros.h>
#include <ros/console.h>

#include <fkie_iop_accesscontrol/RemoteComponent.h>

using namespace iop;


RemoteComponent::RemoteComponent(JausAddress address, jUnsignedByte authority, int timeout)
{
	p_address = address;
	p_timeout = timeout;
	p_authority = authority;
	p_last_request = 0;
	p_last_ack = 0;
	p_has_access = false;
	p_insauth = false;
}

RemoteComponent::~RemoteComponent()
{
}

bool RemoteComponent::operator==(RemoteComponent &value)
{
	return (p_address == value.p_address);
}

bool RemoteComponent::operator!=(RemoteComponent &value)
{
	return !(*this == value);
}

void RemoteComponent::set_timeout(int timeout)
{
	ROS_DEBUG_NAMED("AccessControlClient", "set new timeout %d for %s", timeout, p_address.str().c_str());
	p_timeout = timeout;
}

void RemoteComponent::set_ack(unsigned long secs)
{
	ROS_DEBUG_NAMED("AccessControlClient", "set new ack %lu for %s", secs, p_address.str().c_str());
	p_last_ack = secs;
	p_has_access = true;
	p_insauth = false;
}

void RemoteComponent::set_insufficient_authority()
{
	ROS_DEBUG_NAMED("AccessControlClient", "set insufficient authority to true for %s", p_address.str().c_str());
	p_insauth = true;
}

bool RemoteComponent::timeouted()
{
	if (!p_insauth && p_last_ack < p_last_request && p_last_request - p_last_ack > (unsigned long)p_timeout * 2) {
		ROS_DEBUG_NAMED("AccessControlClient", "timeouted, last req: %lu, last_ack %lu, diff: %lu for %s", p_last_request, p_last_ack, p_last_request - p_last_ack, p_address.str().c_str());
		return true;
	}
	return false;
}

bool RemoteComponent::time_to_send_request(unsigned long deadtime)
{
	double now = ros::WallTime::now().toSec();
	if (p_last_request + (unsigned long)p_timeout < (unsigned long)now + deadtime) {
		p_last_request = now;
		if (p_last_ack == 0) {
			p_last_ack = now;
		}
		return true;
	}
	return false;
}

bool RemoteComponent::has_access()
{
	return p_has_access;
}

bool RemoteComponent::is_insufficient_authority()
{
	return p_insauth;
}

JausAddress RemoteComponent::address()
{
	return p_address;
}

jUnsignedByte RemoteComponent::authority()
{
	return p_authority;
}
