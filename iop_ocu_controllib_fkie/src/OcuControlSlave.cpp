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
#include <iop_ocu_controllib_fkie/OcuControlSlave.h>

unsigned char OcuControlSlave::ACCESS_STATE_NOT_AVAILABLE		= 0;
unsigned char OcuControlSlave::ACCESS_STATE_NOT_CONTROLLED		= 1;
unsigned char OcuControlSlave::ACCESS_STATE_CONTROL_RELEASED	= 2;
unsigned char OcuControlSlave::ACCESS_STATE_CONTROL_ACCEPTED	= 3;
unsigned char OcuControlSlave::ACCESS_STATE_TIMEOUT				= 4;
unsigned char OcuControlSlave::ACCESS_STATE_INSUFFICIENT_AUTHORITY = 5;

unsigned char OcuControlSlave::ACCESS_CONTROL_ON_DEMAND	= 10;
unsigned char OcuControlSlave::ACCESS_CONTROL_REQUEST	= 11;
unsigned char OcuControlSlave::ACCESS_CONTROL_RELEASE	= 12;

OcuControlSlave::OcuControlSlave()
{
	pInit(0, 0, 0);
}

OcuControlSlave::OcuControlSlave(unsigned short subsystem, unsigned char node, unsigned char component)
{
	pInit(subsystem, node, component);
}

OcuControlSlave::~OcuControlSlave(void)
{
}

void OcuControlSlave::pInit(unsigned short subsystem, unsigned char node, unsigned char component)
{
	p_has_control_msg = false;
	ros::NodeHandle nh;
	p_pub_control_feedback = nh.advertise<iop_msgs_fkie::OcuControlFeedback>("ocu_control_feedback", 1, true);
	ros::NodeHandle pnh("~");
	std::string control_addr;
	int authority = 205;
	int access_control = OcuControlSlave::ACCESS_CONTROL_ON_DEMAND;
	if (!pnh.getParam("control_addr", control_addr)) {
		nh.param("control_addr", control_addr, control_addr);
	}
	if (!pnh.getParam("authority", authority)) {
		nh.param("authority", authority, authority);
	}
	if (!pnh.getParam("access_control", access_control)) {
		nh.param("access_control", access_control, access_control);
	}
	if (!control_addr.empty()) {
		std::vector<std::string> strs;
		boost::split(strs, control_addr, boost::is_any_of(".:"));
std::cout << "strs.size()" << strs.size() << "  to parse: " << control_addr.c_str() << std::endl;
		if (strs.size() > 3) {
			throw new std::string("Invalid control_addr parameter: "+ control_addr + "\n");
		} else {
			if (strs.size() > 0) {
				p_msg_control.address.subsystem_id = atoi(strs[0].c_str());
			}
			if (strs.size() > 1) {
				p_msg_control.address.node_id = atoi(strs[1].c_str());
			}
			if (strs.size() > 2) {
				p_msg_control.address.component_id = atoi(strs[2].c_str());
			}
		}
	}
	// try to get the address of the component specified for this slave to avoid discovering
	std::string control_component_addr;
	pnh.param("control_component_addr", control_component_addr, control_component_addr);
	if (!control_component_addr.empty()) {
		std::vector<std::string> strs;
		boost::split(strs, control_component_addr, boost::is_any_of(".:"));
		if (strs.size() > 3) {
			throw new std::string("Invalid control_addr parameter: "+ control_component_addr + "\n");
		} else {
			if (strs.size() > 0) {
				p_control_component.subsystem_id = atoi(strs[0].c_str());
			}
			if (strs.size() > 1) {
				p_control_component.node_id = atoi(strs[1].c_str());
			}
			if (strs.size() > 2) {
				p_control_component.component_id = atoi(strs[2].c_str());
			}
		}
	}
	p_msg_control.authority = authority;
	p_msg_control.access_control = access_control;
	p_msg_feedback.authority = authority;
	ROS_INFO_ONCE_NAMED("OcuControlSlave", "OCU control slave parameter:");
	ROS_INFO_ONCE_NAMED("OcuControlSlave", "	control_addr: %s, decoded to: %d.%d.%d", control_addr.c_str(), p_msg_control.address.subsystem_id, p_msg_control.address.node_id, p_msg_control.address.component_id);
	ROS_INFO_ONCE_NAMED("OcuControlSlave", "	control_component_addr: %s, decoded to: %d.%d.%d", control_component_addr.c_str(), p_control_component.subsystem_id, p_control_component.node_id, p_control_component.component_id);
	ROS_INFO_ONCE_NAMED("OcuControlSlave", "	authority:	%d", authority);
	std::string access_control_str = "ACCESS_CONTROL_ON_DEMAND(10)";
	if (access_control == 1) {
		access_control_str = "ACCESS_CONTROL_REQUEST(11)";
	} else if (access_control == 2) {
		access_control_str = "ACCESS_CONTROL_RELEASE(12)";
	}
	ROS_INFO_ONCE_NAMED("OcuControlSlave", "	access_control: %s", access_control_str.c_str());
	p_msg_feedback.addr_reporter.subsystem_id = subsystem;
	p_msg_feedback.addr_reporter.node_id = node;
	p_msg_feedback.addr_reporter.component_id = component;
	p_msg_feedback.authority = authority;
	// publish the feedback with settings
	p_pub_control_feedback.publish(p_msg_feedback);
	p_sub_control = nh.subscribe<iop_msgs_fkie::OcuControl>("ocu_control", 10, &OcuControlSlave::pRosControl, this);
}

unsigned char OcuControlSlave::get_authority()
{
	return p_msg_control.authority;
}

unsigned short OcuControlSlave::get_control_subsystem()
{
	return p_msg_control.address.subsystem_id;
}

iop_msgs_fkie::JausAddress OcuControlSlave::get_control_platform()
{
	return p_msg_control.address;
}

iop_msgs_fkie::JausAddress OcuControlSlave::get_control_component()
{
	return p_control_component;
}

unsigned char OcuControlSlave::get_access_control()
{
	return p_msg_control.access_control;
}

void OcuControlSlave::set_reporter_address(unsigned short subsystem, unsigned char node, unsigned char component)
{
	if (p_msg_feedback.addr_reporter.subsystem_id != subsystem
			|| p_msg_feedback.addr_reporter.node_id != node
			|| p_msg_feedback.addr_reporter.component_id != component) {
		p_msg_feedback.addr_reporter.subsystem_id = subsystem;
		p_msg_feedback.addr_reporter.node_id = node;
		p_msg_feedback.addr_reporter.component_id = component;
		p_pub_control_feedback.publish(p_msg_feedback);
	}
}

void OcuControlSlave::set_control_address(unsigned short subsystem, unsigned char node, unsigned char component)
{
	if (p_msg_feedback.addr_control.subsystem_id != subsystem
			|| p_msg_feedback.addr_control.node_id != node
			|| p_msg_feedback.addr_control.component_id != component) {
		p_msg_feedback.addr_control.subsystem_id = subsystem;
		p_msg_feedback.addr_control.node_id = node;
		p_msg_feedback.addr_control.component_id = component;
		p_pub_control_feedback.publish(p_msg_feedback);
	}
}

void OcuControlSlave::set_access_state(unsigned char code)
{
	if (p_msg_feedback.access_state != code) {
		p_msg_feedback.access_state = code;
		p_pub_control_feedback.publish(p_msg_feedback);
	}
}

unsigned char OcuControlSlave::get_access_state()
{
	return p_msg_feedback.access_state;
}

void OcuControlSlave::pRosControl(const iop_msgs_fkie::OcuControl::ConstPtr& control)
{
	iop_msgs_fkie::JausAddress a2 = control->address;
	if (!eqaulAddress(p_msg_control.address, a2) || (p_msg_control.authority != control->authority)) {

		p_msg_control.address = a2;
		p_msg_control.authority = control->authority;
		if (!p_class_control_platform_callback.empty()) {
			p_class_control_platform_callback(p_msg_control.address.subsystem_id, p_msg_control.address.node_id,
					p_msg_control.address.component_id, p_msg_control.authority);
		}
	}
	if (p_msg_control.access_control != control->access_control) {
		p_msg_control.access_control = control->access_control;
		if (!p_class_access_control_callback.empty()) {
			p_class_access_control_callback(p_msg_control.access_control);
		}
	}
}

bool OcuControlSlave::eqaulAddress(iop_msgs_fkie::JausAddress &a1, iop_msgs_fkie::JausAddress &a2)
{
	return ((a1.subsystem_id == a2.subsystem_id)
			&& (a1.node_id == a2.node_id)
			&& (a1.component_id == a2.component_id));
}
