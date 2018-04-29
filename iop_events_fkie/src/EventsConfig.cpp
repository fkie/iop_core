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


#include <iop_events_fkie/EventsConfig.h>
#include <iop_component_fkie/iop_config.h>

using namespace iop;


GNU_CONST_STATIC_FLOAT_DECLARATION float EventsConfig::MINIMUM_RATE = 0.1f;
GNU_CONST_STATIC_FLOAT_DECLARATION float EventsConfig::MAXIMUM_RATE = 25.0f;
GNU_CONST_STATIC_FLOAT_DECLARATION float EventsConfig::RATE_PRECISION = 0.1f;

EventsConfig::EventsConfig()
{
	p_default_timeout = 1;
	iop::Config cfg("~Events");
	cfg.param("events_timeout", p_default_timeout, p_default_timeout);
}

EventsConfig::EventsConfig(EventsConfig const& from)
{
	p_default_timeout = from.p_default_timeout;
}

const EventsConfig& EventsConfig::operator=(const EventsConfig& from)
{
	if (this != &from) {
		this->p_default_timeout = from.p_default_timeout;
	}
	return *this;
}

int EventsConfig::get_timeout()
{
	return p_default_timeout;
}
