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

#include <string>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/timer.hpp>
#include "RemoteComponent.h"


namespace iop
{

class RemoteComponentList {
public:
	RemoteComponentList(int64_t default_timeout=5);
	~RemoteComponentList();

	bool create(JausAddress address, jUnsignedByte authority);
	bool remove(JausAddress address);
	bool isin(JausAddress address);
	void set_ack(JausAddress address, int64_t secs);
	void set_insufficient_authority(JausAddress address);
	void set_timeout(JausAddress address, int64_t timeout);
	bool has_access(JausAddress address);
	std::vector<std::shared_ptr<iop::RemoteComponent> > time_to_send_request(int64_t deadtime=2);
	std::vector<std::shared_ptr<iop::RemoteComponent> > timeouted();

protected:
	rclcpp::Logger logger;
	typedef std::recursive_mutex mutex_type;
	typedef std::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;

	int p_default_timeout;
	// iop::Timer p_timer;
	std::map<JausAddress, std::shared_ptr<iop::RemoteComponent> > p_components;

	//void p_timeout(const ros::TimerEvent& event);
};

}

#endif
