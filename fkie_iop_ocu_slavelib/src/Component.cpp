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
#include <fkie_iop_ocu_slavelib/Component.h>

using namespace iop::ocu;


Component::Component(JausAddress address)
{
	p_address = address;
	p_access_state = Component::ACCESS_STATE_NOT_AVAILABLE;
	p_access_control = Component::ACCESS_CONTROL_RELEASE;
	p_authority = 205;
}

Component::~Component(void)
{
}

bool Component::set_state(unsigned char access_state)
{
	bool changed = false;
	if (p_access_state != access_state) {
		changed = true;
		p_access_state = access_state;
	}
	return changed;
}

unsigned char Component::get_state()
{
	return p_access_state;
}

unsigned char Component::get_authority()
{
	return p_authority;
}

bool Component::set_authority(unsigned char authority)
{
	bool changed = false;
	if (p_authority != authority) {
		p_authority = authority;
		changed = true;
	}
	return changed;
}

unsigned char Component::get_access_control()
{
	return p_access_control;
}

JausAddress Component::get_address() const
{
	return p_address;
}

bool Component::match(JausAddress expr)
{
	return match_address(p_address, expr);
}

bool Component::operator==(const Component &other)
{
	return (p_address.get() == other.get_address().get());
}

bool Component::set_access_control(unsigned char access_control)
{
	bool changed = false;
	if (p_access_control != access_control) {
		p_access_control = access_control;
		changed = true;
	}
	return changed;
}


