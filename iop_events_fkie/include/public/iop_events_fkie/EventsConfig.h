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


#ifndef EVENTSCONFIG_H
#define EVENTSCONFIG_H

#include <ros/ros.h>
#include <string>

namespace iop
{

#if __GNUC__ > 5
 #define GNU_CONST_STATIC_FLOAT_DECLARATION constexpr
#else
 #define GNU_CONST_STATIC_FLOAT_DECLARATION const
#endif

class EventsConfig {
public:
	static GNU_CONST_STATIC_FLOAT_DECLARATION float MINIMUM_RATE;
	static GNU_CONST_STATIC_FLOAT_DECLARATION float MAXIMUM_RATE;
	static GNU_CONST_STATIC_FLOAT_DECLARATION float RATE_PRECISION;
	EventsConfig();
	EventsConfig(EventsConfig const& from);
	const EventsConfig& operator=(const EventsConfig& from);
	int get_timeout();

protected:
	int p_default_timeout;
};

};

#endif
