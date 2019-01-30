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

#ifndef OCU_COMPONENT_H_
#define OCU_COMPONENT_H_

#include <Transport/JausAddress.h>
#include <fkie_iop_msgs/JausAddress.h>

namespace iop
{

namespace ocu
{

class Component {
public:

	static const unsigned char ACCESS_STATE_NOT_AVAILABLE		= 0;
	static const unsigned char ACCESS_STATE_NOT_CONTROLLED		= 1;
	static const unsigned char ACCESS_STATE_CONTROL_RELEASED	= 2;
	static const unsigned char ACCESS_STATE_CONTROL_ACCEPTED	= 3;
	static const unsigned char ACCESS_STATE_TIMEOUT				= 4;
	static const unsigned char ACCESS_STATE_INSUFFICIENT_AUTHORITY	= 5;
	static const unsigned char ACCESS_STATE_MONITORING			= 6;

	static const unsigned char ACCESS_CONTROL_RELEASE = 10;
	static const unsigned char ACCESS_CONTROL_MONITOR = 11;
	static const unsigned char ACCESS_CONTROL_REQUEST = 12;

	Component(JausAddress address);
	~Component();
	bool set_state(unsigned char access_state);
	unsigned char get_state();
	unsigned char get_authority();
	bool set_authority(unsigned char authority);
	unsigned char get_access_control();
	JausAddress get_address() const;
	bool match(JausAddress expr);
	bool set_access_control(unsigned char access_control);
	bool operator==(const Component &other);
protected:
	JausAddress p_address;
	unsigned char p_access_state;
	unsigned char p_access_control;
	unsigned char p_authority;
};

};

};

#endif
