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


#ifndef IOPREMOTECOMPONENTLIST_H
#define IOPREMOTECOMPONENTLIST_H

#include "Transport/JausTransport.h"

#include <boost/thread/recursive_mutex.hpp>
#include <ros/ros.h>
#include <string>
#include "RemoteComponent.h"


namespace iop
{

class RemoteComponentList {
public:
	RemoteComponentList(int default_timeout=5);
	~RemoteComponentList();

	bool create(JausAddress address, jUnsignedByte authority);
	bool remove(JausAddress address);
	bool isin(JausAddress address);
	void set_ack(JausAddress address, unsigned long secs);
	void set_insufficient_authority(JausAddress address);
	void set_timeout(JausAddress address, int timeout);
	bool has_access(JausAddress address);
	std::vector<boost::shared_ptr<iop::RemoteComponent> > time_to_send_request(unsigned long deadtime=2);
	std::vector<boost::shared_ptr<iop::RemoteComponent> > timeouted();

protected:
	typedef boost::recursive_mutex mutex_type;
	typedef boost::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;

	int p_default_timeout;
	ros::NodeHandle p_nh;
	ros::Timer p_timeout_timer;
	std::map<JausAddress, boost::shared_ptr<iop::RemoteComponent> > p_components;

	//void p_timeout(const ros::TimerEvent& event);
};

};

#endif
