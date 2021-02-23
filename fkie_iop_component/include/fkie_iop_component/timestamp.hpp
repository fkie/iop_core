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

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>


namespace iop {

class Timestamp {

public:
	Timestamp(uint64_t days, uint64_t hours, uint64_t minutes, uint64_t seconds, uint64_t milliseconds, rclcpp::Time ros_now, bool use_remote_time=false) {
		this->days = days;
		this->hours = hours;
		this->minutes = minutes;
		this->seconds = seconds;
		this->milliseconds = milliseconds;
		uint64_t now_sec = ros_now.seconds();
		uint64_t days_mod = now_sec - now_sec % 86400;
		ros_time = rclcpp::Time(days_mod + seconds + minutes * 60 + hours * 3600 + days * 86400, milliseconds * 1000000);
		if (! use_remote_time) {
			// THIS IS A HACK: avoid problems with not synchronized hosts
			ros_time = ros_now;
		}
	}

	Timestamp(rclcpp::Time ros_time) {
		this->ros_time = ros_time;
		uint64_t days_mod = int64_t(ros_time.seconds()) % 86400;
		uint64_t days = (ros_time.seconds() - days_mod) / 86400;
		this->days = days;
		uint64_t hours_mod = days_mod % 3600;
		uint64_t hours = (days_mod - hours_mod) / 3600;
		this->hours = hours;
		uint64_t minutes_mod = hours_mod % 60;
		uint64_t minutes = (hours_mod - minutes_mod) / 60;
		this->minutes = minutes;
		uint64_t seconds = minutes_mod % 60;
		this->seconds = seconds;
		this->milliseconds = ros_time.nanoseconds() / 1000000;
	}

	rclcpp::Time ros_time;
	uint64_t days;
	uint64_t hours;
	uint64_t minutes;
	uint64_t seconds;
	uint64_t milliseconds;

};

}


#endif // TIMESTAMP_H
