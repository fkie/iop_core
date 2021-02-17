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


#include <string>
#include <fkie_iop_events/EventsConfig.h>
#include <fkie_iop_component/iop_config.hpp>

using namespace iop;

#if __GNUC__ > 5
#else
const float EventsConfig::MINIMUM_RATE = 0.1f;
const float EventsConfig::MAXIMUM_RATE = 25.0f;
const float EventsConfig::RATE_PRECISION = 0.1f;
#endif

EventsConfig::EventsConfig(int64_t timeout)
{
	p_default_timeout = timeout;
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

void EventsConfig::set_timeout(int64_t timeout)
{
	p_default_timeout = timeout;
}

int64_t EventsConfig::get_timeout()
{
	return p_default_timeout;
}
