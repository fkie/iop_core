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

#ifndef OCUCONTROLMASTER_H
#define OCUCONTROLMASTER_H

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <iop_msgs_fkie/OcuControl.h>
#include <iop_msgs_fkie/OcuControlFeedback.h>

class OcuControlMaster
{
 public:
	static unsigned char ACCESS_CONTROL_ON_DEMAND;
	static unsigned char ACCESS_CONTROL_REQUEST;
	static unsigned char ACCESS_CONTROL_RELEASE;

	OcuControlMaster();
	~OcuControlMaster(void);

	void set_control(unsigned short subsystem, unsigned char node=0, unsigned char component=0, unsigned char authority=205, unsigned char access_control=0);
	void set_access_control(unsigned char value);

	template<class T>
	void set_feedback_handler(void(T::*handler)(const iop_msgs_fkie::OcuControlFeedback::ConstPtr &), T*obj);

 protected:
	boost::function<void (const iop_msgs_fkie::OcuControlFeedback::ConstPtr &)> p_class_feedback_callback;
	// create messages that are used to published control message
	iop_msgs_fkie::OcuControl p_msg_control;
	ros::Publisher p_pub_control;
	ros::Subscriber p_sub_control_feedback;

	void pRosControlFeedback(const iop_msgs_fkie::OcuControlFeedback::ConstPtr& control_feedback);
};

template<class T>
void OcuControlMaster::set_feedback_handler(void(T::*handler)(const iop_msgs_fkie::OcuControlFeedback::ConstPtr &), T*obj) {
	bool subscribe_after = p_class_feedback_callback.empty();
	p_class_feedback_callback = boost::bind(handler, obj, _1);
	if (subscribe_after) {
		ros::NodeHandle nh;
		p_sub_control_feedback = nh.subscribe<iop_msgs_fkie::OcuControlFeedback>("ocu_control_feedback", 1, &OcuControlMaster::pRosControlFeedback, this);
	}
}


#endif // OCUCONTROLMASTER_H
