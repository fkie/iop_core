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


#include <ros/ros.h>
#include <ros/console.h>

#include <iop_events_fkie/InternalEvent.h>
#include <iop_events_fkie/InternalEventList.h>

using namespace iop;

InternalEvent::InternalEvent(InternalEventList* event_list)
{
	p_event_list = event_list;
	p_last_update = ros::Time(0);
	p_query_msg_id = 0;
	p_request_id = 255;
	p_event_id = 255;
	p_event_type = 0;
	p_event_rate = 0.0;
	seq_nr = 0;
	p_supports_on_change = false;
	p_supports_periodic = false;
	p_error_code = 4;
	p_error_msg = "Event was not initialized";
	p_initialized = false;
	p_last_report = NULL;
}

InternalEvent::InternalEvent(InternalEventList* event_list, jUnsignedByte request_id, jUnsignedShortInteger query_msg_id, jUnsignedByte event_type, double event_rate)
// : InternalEvent(event_list)  warning: delegating constructors only available with -std=c++11 or -std=gnu++11
{
	p_event_list = event_list;
	p_last_update = ros::Time(0);
	p_query_msg_id = 0;
	p_request_id = 255;
	p_event_id = 255;
	p_event_type = 0;
	p_event_rate = 0.0;
	seq_nr = 0;
	p_supports_on_change = false;
	p_supports_periodic = false;
	p_error_code = 4;
	p_error_msg = "Event was not initialized";
	p_initialized = false;

	p_request_id = request_id;
	p_query_msg_id = query_msg_id;
	p_event_type = event_type;
	p_event_rate = event_rate;
	p_last_report = NULL;
	p_is_event_supported(query_msg_id, event_type, event_rate);
}

InternalEvent::InternalEvent(InternalEventList* event_list, jUnsignedByte event_id, jUnsignedByte request_id, jUnsignedShortInteger query_msg_id, jUnsignedByte event_type, double event_rate, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage query_msg, JausAddress requestor)
//: InternalEvent(event_list, request_id, query_msg_id) warning: delegating constructors only available with -std=c++11 or -std=gnu++11
{
	p_event_list = event_list;
	p_last_update = ros::Time(0);
	p_query_msg_id = 0;
	p_request_id = 255;
	p_event_id = 255;
	p_event_type = 0;
	p_event_rate = 0.0;
	seq_nr = 0;
	p_supports_on_change = false;
	p_supports_periodic = false;
	p_error_code = 4;
	p_error_msg = "Event was not initialized";
	p_initialized = false;

	p_request_id = request_id;
	p_query_msg_id = query_msg_id;
	p_event_type = event_type;
	p_event_rate = event_rate;
	p_last_report = NULL;

	if (p_is_event_supported(query_msg_id, event_type, event_rate)) {
		p_request_id = request_id;
		p_last_update = ros::Time::now();
		p_event_id = event_id;
		p_query_msg = query_msg;
		p_query_msg_id = query_msg_id;
		this->requestor = requestor;
		p_supports_on_change = p_event_list->supports_on_change(query_msg_id);
		p_supports_periodic = p_event_list->supports_periodic(query_msg_id);
		update(event_id, p_query_msg, query_msg_id, requestor, request_id, event_type, event_rate);
		p_initialized = true;
	}
}

InternalEvent::~InternalEvent()
{
	p_timer_stop();
}

bool InternalEvent::operator==(InternalEvent &value)
{
	return p_event_id == value.p_event_id;
}

bool InternalEvent::operator!=(InternalEvent &value)
{
	return !(*this == value);
}

void InternalEvent::new_report_available(JTS::Message *report, bool send_if_possible)
{
	if (report != NULL) {
		if (p_event_type == 1) {
			// send on change
			if (send_if_possible) {
				p_send_as_event(*report, requestor);
			} else {
				p_last_report = report;
			}
		} else {
			if (p_timeout_timer.isValid()) {
				// do nothing, the report will be send on next timer call
			} else {
				// report is available and we have periodic event -> create timer
				p_timer_start();
			}
		}
	} else {
		p_timer_stop();
	}
}

void InternalEvent::send_last_report(){
	if (p_last_report != NULL) {
		if (p_event_type == 1) {
			ROS_DEBUG_NAMED("Events", "  send available report: %#x", p_last_report->getID());
			p_send_as_event(*p_last_report, requestor);
			p_last_report = NULL;
		}
	}
}

urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage &InternalEvent::get_query()
{
	return p_query_msg;
}

void InternalEvent::send_report(JTS::Message &report, unsigned short id)
{
	if (p_event_type == 1) {
		// send on change
		p_send_as_event(report, requestor);
	} else {
		if (p_can_send(id)) {
			p_send_as_event(report, requestor);
			// save the last send timestamp for a given ID
			p_last_send[id] = ros::Time::now();
		}
	}
}

void InternalEvent::update(jUnsignedByte event_id, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage query_msg, jUnsignedShortInteger query_msg_id, JausAddress requestor, jUnsignedByte request_id, jUnsignedByte event_type, double event_rate)
{
	if (p_event_id == event_id) {
		p_last_update = ros::Time::now();
		p_request_id = request_id;
		p_query_msg = query_msg;
		p_query_msg_id = query_msg_id;
		if (!p_initialized) {
			p_event_type = event_type;
			p_event_rate = event_rate;
			if (event_type == 0) {
				p_timer_start();
			}
		} else {
			// it is initialized, check for changed event type or rate
			if (p_event_type != event_type) {
				p_event_type = event_type;
				p_event_rate = event_rate;
				if (event_type == 0) {
					p_timer_start();
				} else if (event_type == 1) {
					p_timer_stop();
				}
			} else if (p_event_type == 0 && p_event_rate != event_rate) {
				p_event_rate = event_rate;
				p_timer_stop();
				p_timer_start();
			}
		}
	}
}

void InternalEvent::set_error(jUnsignedByte code, std::string msg)
{
	p_error_code = code;
	p_error_msg = msg;
}

bool InternalEvent::p_is_event_supported(jUnsignedShortInteger query_msg_id, jUnsignedByte p_event_type, double p_event_rate)
{
	if (p_event_type == 0 && !p_event_list->supports_periodic(query_msg_id)) {
		set_error(1, "Periodic events not supported");
		return false;
	} else if (p_event_type == 1 && !p_event_list->supports_on_change(query_msg_id)) {
		set_error(2, "Change based events not supported");
		return false;
	} else if (p_event_type == 0 && !p_event_list->supports_rate(query_msg_id, p_event_rate)) {
		set_error(4, "rate not supported");
		return false;
	} else if (!p_event_list->supports_message(query_msg_id)) {
		set_error(5, "Message not supported");
		return false;
	}
	set_error(0);
	return true;
}

void InternalEvent::p_send_as_event(JTS::Message &report, JausAddress &address)
{
	jUnsignedInteger len = report.getSize();
	if (len > 65536) {
		ROS_WARN_NAMED("Events", "large message detected, size: %d", len);
	}
	unsigned char bytes[len];
	report.encode(bytes);
	urn_jaus_jss_core_Events::Event event;
	event.getBody()->getEventRec()->setSequenceNumber(seq_nr);
	event.getBody()->getEventRec()->setEventID(get_event_id());
	event.getBody()->getEventRec()->getReportMessage()->set(len, bytes);
	seq_nr++;
	p_event_list->get_jr_handler()->sendJausMessage(event, address);
}

void InternalEvent::timeout(const ros::TimerEvent& event)
{
	JTS::Message* report = p_event_list->get_report(p_query_msg_id);
	if (report != NULL) {
		p_send_as_event(*report, requestor);
		// save the last send timestamp for a given ID
		p_last_send[255] = ros::Time::now();
	}
}

void InternalEvent::p_timer_stop()
{
	if (p_timeout_timer.isValid()) {
		ROS_DEBUG_NAMED("Events", "stop event timer for report %#x with rate %.2f to %s", p_query_msg_id, p_event_rate, requestor.str().c_str());
		p_timeout_timer.stop();
	}
}

void InternalEvent::p_timer_start()
{
	if (p_event_list->get_report(p_query_msg_id) != NULL) {
		if (p_event_rate > 0) {
			ROS_DEBUG_NAMED("Events", "start event timer for %#x with rate %.2f to %s", p_query_msg_id, p_event_rate, requestor.str().c_str());
			p_timeout_timer = p_nh.createTimer(ros::Duration(1.0 / p_event_rate), &InternalEvent::timeout, this);
		}
	}
}

bool InternalEvent::p_can_send(unsigned short id)
{
	std::map<jUnsignedShortInteger, ros::Time>::iterator res = p_last_send.find(id);
	if (res != p_last_send.end()) {
		ros::Duration last = ros::Time::now() - res->second;
		ros::Duration interval = ros::Duration(1.0 / p_event_rate) - ros::Duration(iop::EventsConfig::RATE_PRECISION);
		return (last >= interval);
	}
	return true;
}
