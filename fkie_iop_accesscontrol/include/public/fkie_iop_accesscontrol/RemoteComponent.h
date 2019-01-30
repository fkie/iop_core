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


#ifndef IOPREMOTECOMPONENT_H
#define IOPREMOTECOMPONENT_H


#include <ros/ros.h>
#include <string>
#include "Transport/JausTransport.h"


namespace iop
{

class RemoteComponent {

public:
	/** A */
	RemoteComponent(JausAddress address, jUnsignedByte authority, int timeout);
	~RemoteComponent();

	bool operator==(RemoteComponent &value);
	bool operator!=(RemoteComponent &value);

	void set_timeout(int timeout);
	void set_ack(unsigned long secs);
	void set_insufficient_authority();
	bool timeouted();
	bool time_to_send_request(unsigned long deadtime=2);
	bool has_access();
	bool is_insufficient_authority();
	JausAddress address();
	jUnsignedByte authority();

protected:
	JausAddress p_address;
	int p_timeout;
	unsigned long p_last_request;
	unsigned long p_last_ack;
	jUnsignedByte p_authority;
	bool p_has_access;
	bool p_insauth;

private:
	RemoteComponent(const RemoteComponent& that);
	const RemoteComponent& operator=(const RemoteComponent& from);
};

};

#endif
