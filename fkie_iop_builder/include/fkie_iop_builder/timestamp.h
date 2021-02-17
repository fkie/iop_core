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


#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <string>
#include <ros/ros.h>
#include <ros/time.h>


namespace iop {

static bool use_remote_time = false;
static bool parameter_initialized = false;

class Timestamp {

public:
	Timestamp(unsigned int days, unsigned int hours, unsigned int minutes, unsigned int seconds, unsigned int milliseconds) {
		this->days = days;
		this->hours = hours;
		this->minutes = minutes;
		this->seconds = seconds;
		this->milliseconds = milliseconds;
		unsigned int now_sec = ros::Time::now().sec;
		unsigned int days_mod = now_sec - now_sec % 86400;
		ros_time.sec =  days_mod + seconds + minutes * 60 + hours * 3600 + days * 86400;
		ros_time.nsec = milliseconds * 1000000;
		// THIS IS A HACK: avoid problems with not synchronized hosts
		if (! parameter_initialized) {
			ros::NodeHandle nh;
			nh.param("iop_use_remote_time", use_remote_time, use_remote_time);
			ROS_INFO_STREAM("ROS param: " << "iop_use_remote_time" << " = " << use_remote_time);
			parameter_initialized = true;
		}
		if (! use_remote_time) {
			ros_time = ros::Time::now();
		}
	}

	Timestamp(ros::Time ros_time) {
		this->ros_time = ros_time;
		unsigned int days_mod = ros_time.sec % 86400;
		unsigned int days = (ros_time.sec - days_mod) / 86400;
		this->days = days;
		unsigned int hours_mod = days_mod % 3600;
		unsigned int hours = (days_mod - hours_mod) / 3600;
		this->hours = hours;
		unsigned int minutes_mod = hours_mod % 60;
		unsigned int minutes = (hours_mod - minutes_mod) / 60;
		this->minutes = minutes;
		unsigned int seconds = minutes_mod % 60;
		this->seconds = seconds;
		this->milliseconds = ros_time.nsec / 1000000;
	}

	ros::Time ros_time;
	unsigned int days;
	unsigned int hours;
	unsigned int minutes;
	unsigned int seconds;
	unsigned int milliseconds;

};

};


#endif // TIMESTAMP_H
