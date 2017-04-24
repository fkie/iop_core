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

#ifndef OCUCONTROLSLAVE_H
#define OCUCONTROLSLAVE_H

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <iop_msgs_fkie/OcuControl.h>
#include <iop_msgs_fkie/OcuControlFeedback.h>
#include <iop_msgs_fkie/JausAddress.h>

class OcuControlSlave
{
 public:
	static unsigned char ACCESS_STATE_NOT_AVAILABLE;
	static unsigned char ACCESS_STATE_NOT_CONTROLLED;
	static unsigned char ACCESS_STATE_CONTROL_RELEASED;
	static unsigned char ACCESS_STATE_CONTROL_ACCEPTED;
	static unsigned char ACCESS_STATE_TIMEOUT;
	static unsigned char ACCESS_STATE_INSUFFICIENT_AUTHORITY;

	static unsigned char ACCESS_CONTROL_ON_DEMAND;
	static unsigned char ACCESS_CONTROL_REQUEST;
	static unsigned char ACCESS_CONTROL_RELEASE;


	OcuControlSlave();
	OcuControlSlave(unsigned short subsystem, unsigned char node, unsigned char component);
	~OcuControlSlave(void);

	unsigned char get_authority();
	unsigned short get_control_subsystem();
	iop_msgs_fkie::JausAddress get_control_platform();
	iop_msgs_fkie::JausAddress get_control_component();
	unsigned char get_access_control();

	template<class T>
	void set_control_platform_handler(void(T::*handler)(unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority), T*obj);
	template<class T>
	void set_control_component_handler(void(T::*handler)(unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority), T*obj);
	template<class T>
	void set_access_control_handler(void(T::*handler)(unsigned char value), T*obj);
	void set_reporter_address(unsigned short subsystem, unsigned char node, unsigned char component);
	void set_control_address(unsigned short subsystem, unsigned char node, unsigned char component);
	void set_access_state(unsigned char code);
	unsigned char get_access_state();
	bool eqaulAddress(iop_msgs_fkie::JausAddress &a1, iop_msgs_fkie::JausAddress &a2);

 protected:
	boost::function<void (unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority)> p_class_control_platform_callback;
	boost::function<void (unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority)> p_class_control_component_callback;
	boost::function<void (unsigned char value)> p_class_access_control_callback;
	bool p_has_control_msg;
	iop_msgs_fkie::JausAddress p_control_component;
	// create messages that are used to published control message
	iop_msgs_fkie::OcuControl p_msg_control;
	iop_msgs_fkie::OcuControlFeedback p_msg_feedback;
	ros::Publisher p_pub_control_feedback;
	ros::Subscriber p_sub_control;

	void pInit(unsigned short subsystem, unsigned char node, unsigned char component);
	void pRosControl(const iop_msgs_fkie::OcuControl::ConstPtr& control);
};

template<class T>
void OcuControlSlave::set_control_platform_handler(void(T::*handler)(unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority), T*obj) {
	p_class_control_platform_callback = boost::bind(handler, obj, _1, _2, _3, _4);
	p_class_control_platform_callback(p_msg_control.address.subsystem_id, p_msg_control.address.node_id,
			p_msg_control.address.component_id, p_msg_control.authority);
}

template<class T>
void OcuControlSlave::set_control_component_handler(void(T::*handler)(unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority), T*obj) {
	p_class_control_component_callback = boost::bind(handler, obj, _1, _2, _3, _4);
	if (p_control_component.component_id != 0) {
		p_class_control_component_callback(p_control_component.subsystem_id, p_control_component.node_id,
				p_control_component.component_id, p_msg_control.authority);
	}
}

template<class T>
void OcuControlSlave::set_access_control_handler(void(T::*handler)(unsigned char value), T*obj) {
	p_class_access_control_callback = boost::bind(handler, obj, _1);
	p_class_access_control_callback(p_msg_control.access_control);
}


#endif // OCUCONTROLSLAVE_H
