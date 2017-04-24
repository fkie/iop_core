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
#include <iop_ocu_controllib_fkie/OcuControlMaster.h>

unsigned char OcuControlMaster::ACCESS_CONTROL_ON_DEMAND	= 10;
unsigned char OcuControlMaster::ACCESS_CONTROL_REQUEST		= 11;
unsigned char OcuControlMaster::ACCESS_CONTROL_RELEASE		= 12;

OcuControlMaster::OcuControlMaster()
{
	ros::NodeHandle nh;
	p_pub_control = nh.advertise<iop_msgs_fkie::OcuControl>("ocu_control", 1, true);
	ros::NodeHandle pnh("~");
	std::string control_addr;
	int authority = 205;
	int access_control = OcuControlMaster::ACCESS_CONTROL_ON_DEMAND;
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
		if (strs.size() > 3) {
			throw new std::string("Invalid control_addr parameter: "+ control_addr + "\n");
		} else {
			try {
				p_msg_control.address.subsystem_id = atoi(strs[0].c_str());
				p_msg_control.address.node_id = atoi(strs[1].c_str());
				p_msg_control.address.subsystem_id = atoi(strs[2].c_str());
			} catch (...) {
			}
		}
	}
	p_msg_control.authority = authority;
	p_msg_control.access_control = access_control;
	ROS_INFO_NAMED("OcuControlMaster", "OCU control master parameter:");
	ROS_INFO_NAMED("OcuControlMaster", "	control_addr: %s, decoded to: %d.%d.%d", control_addr.c_str(), p_msg_control.address.subsystem_id, p_msg_control.address.node_id, p_msg_control.address.subsystem_id);
	ROS_INFO_NAMED("OcuControlMaster", "	authority:	%d", authority);
	std::string access_control_str = "ACCESS_CONTROL_ON_DEMAND(0)";
	if (access_control == 1) {
		access_control_str = "ACCESS_CONTROL_REQUEST(1)";
	} else if (access_control == 2) {
		access_control_str = "ACCESS_CONTROL_RELEASE(2)";
	}
	ROS_INFO_NAMED("OcuControlMaster", "	access_control: %s", access_control_str.c_str());
	p_pub_control.publish(p_msg_control);
	p_sub_control_feedback = nh.subscribe<iop_msgs_fkie::OcuControlFeedback>("ocu_control_feedback", 10, &OcuControlMaster::pRosControlFeedback, this);
}

OcuControlMaster::~OcuControlMaster(void)
{

}

void OcuControlMaster::set_control(unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority, unsigned char access_control)
{
	if (p_msg_control.address.subsystem_id != subsystem
			|| p_msg_control.address.node_id != node
			|| p_msg_control.address.subsystem_id != component
			|| p_msg_control.authority != authority
			|| p_msg_control.access_control != access_control) {
		p_msg_control.address.subsystem_id = subsystem;
		p_msg_control.address.node_id = node;
		p_msg_control.address.subsystem_id = component;
		p_msg_control.authority = authority;
		p_msg_control.access_control = access_control;
		p_pub_control.publish(p_msg_control);
	}
}

void OcuControlMaster::set_access_control(unsigned char value)
{
	if (p_msg_control.access_control != value) {
		p_msg_control.access_control = value;
		p_pub_control.publish(p_msg_control);
	}
}

void OcuControlMaster::pRosControlFeedback(const iop_msgs_fkie::OcuControlFeedback::ConstPtr& control_feedback)
{
	if (!p_class_feedback_callback.empty()) {
		p_class_feedback_callback(control_feedback);
	}
}
