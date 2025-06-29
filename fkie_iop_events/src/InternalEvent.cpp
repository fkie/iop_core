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


#include <fkie_iop_events/InternalEvent.h>
#include <fkie_iop_events/InternalEventList.h>

using namespace iop;

InternalEvent::InternalEvent(rclcpp::Logger& logger, InternalEventList* event_list)
: logger(logger),
  p_timer(std::chrono::seconds(1), std::bind(&InternalEvent::timeout, this), false)
{
	p_event_list = event_list;
	p_last_update = ChronoSysTP::min();
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

InternalEvent::InternalEvent(rclcpp::Logger& logger, InternalEventList* event_list, jUnsignedByte request_id, jUnsignedShortInteger query_msg_id, jUnsignedByte event_type, double event_rate)
// : InternalEvent(event_list)  warning: delegating constructors only available with -std=c++11 or -std=gnu++11
: logger(logger),
  p_timer(std::chrono::seconds(1), std::bind(&InternalEvent::timeout, this), false)
{
	p_event_list = event_list;
	p_last_update = ChronoSysTP::min();
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
	p_timer.set_rate(event_rate);
	p_is_event_supported(query_msg_id, event_type, event_rate);
}

InternalEvent::InternalEvent(rclcpp::Logger& logger, InternalEventList* event_list, jUnsignedByte event_id, jUnsignedByte request_id, jUnsignedShortInteger query_msg_id, jUnsignedByte event_type, double event_rate, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage query_msg, JausAddress requestor)
//: InternalEvent(event_list, request_id, query_msg_id) warning: delegating constructors only available with -std=c++11 or -std=gnu++11
: logger(logger),
  p_timer(std::chrono::seconds(1), std::bind(&InternalEvent::timeout, this), false)
{
	p_event_list = event_list;
	p_last_update = ChronoSysTP::min();
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
	p_timer.set_rate(event_rate);

	if (p_is_event_supported(query_msg_id, event_type, event_rate)) {
		p_request_id = request_id;
		p_last_update = std::chrono::steady_clock::now();
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
	lock_type lock(p_mutex);
	if (report != NULL) {
		if (p_event_type == 1) {
			// send on change
			if (send_if_possible) {
				p_send_as_event(*report, requestor);
			} else {
				p_last_report = report;
			}
		} else {
			if (p_timer.is_running()) {
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
	lock_type lock(p_mutex);
	if (p_last_report != NULL) {
		if (p_event_type == 1) {
			RCLCPP_DEBUG(logger, "  send available report: %#x", p_last_report->getID());
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
	lock_type lock(p_mutex);
	if (p_event_type == 1) {
		// send on change
		p_send_as_event(report, requestor);
	} else {
		if (p_can_send(id)) {
			p_send_as_event(report, requestor);
			// save the last send timestamp for a given ID
			p_last_send[id] = std::chrono::steady_clock::now();
		}
	}
}

void InternalEvent::update(jUnsignedByte event_id, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage query_msg, jUnsignedShortInteger query_msg_id, JausAddress /* requestor */, jUnsignedByte request_id, jUnsignedByte event_type, double event_rate)
{
	lock_type lock(p_mutex);
	if (p_event_id == event_id) {
		p_last_update = std::chrono::steady_clock::now();
		p_request_id = request_id;
		p_query_msg = query_msg;
		p_query_msg_id = query_msg_id;
		if (!p_initialized) {
			p_event_type = event_type;
			p_event_rate = event_rate;
			p_timer.set_rate(event_rate);
			if (event_type == 0) {
				p_timer_start();
			}
		} else {
			// it is initialized, check for changed event type or rate
			if (p_event_type != event_type) {
				p_event_type = event_type;
				p_event_rate = event_rate;
				p_timer.set_rate(event_rate);
				if (event_type == 0) {
					p_timer_start();
				} else if (event_type == 1) {
					p_timer_stop();
				}
			} else if (p_event_type == 0 && p_event_rate != event_rate) {
				p_event_rate = event_rate;
				p_timer.set_rate(event_rate);
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
	if (!p_event_list->supports_message(query_msg_id)) {
		set_error(5, "Message not supported");
		return false;
	} else if (p_event_type == 0 && !p_event_list->supports_periodic(query_msg_id)) {
		set_error(1, "Periodic events not supported");
		return false;
	} else if (p_event_type == 1 && !p_event_list->supports_on_change(query_msg_id)) {
		set_error(2, "Change based events not supported");
		return false;
	} else if (p_event_type == 0 && !p_event_list->supports_rate(query_msg_id, p_event_rate)) {
		set_error(4, "rate not supported");
		return false;
	} 
	set_error(0);
	return true;
}

void InternalEvent::p_send_as_event(JTS::Message &report, JausAddress &address)
{
	const jUnsignedInteger len = report.getSize();
	if (len > 65536) {
		RCLCPP_WARN(logger, "large message detected, size: %d", len);
	}
	unsigned char* bytes = new unsigned char[len];
	report.encode(bytes);
	urn_jaus_jss_core_Events::Event event;
	event.getBody()->getEventRec()->setSequenceNumber(seq_nr);
	event.getBody()->getEventRec()->setEventID(get_event_id());
	event.getBody()->getEventRec()->getReportMessage()->set(len, bytes);
	seq_nr++;
	p_event_list->get_jr_handler()->sendJausMessage(event, address);
	delete[] bytes;
}

void InternalEvent::timeout()
{
	lock_type lock(p_mutex);
	JTS::Message* report = p_event_list->get_report(p_query_msg_id);
	if (report != NULL) {
		p_send_as_event(*report, requestor);
		// save the last send timestamp for a given ID
		p_last_send[255] = std::chrono::steady_clock::now();
	}
}

void InternalEvent::p_timer_stop()
{
	if (p_timer.is_running()) {
		RCLCPP_DEBUG(logger, "stop event timer for report %#x with rate %.2f to %s", p_query_msg_id, p_event_rate, requestor.str().c_str());
		p_timer.stop();
	}
}

void InternalEvent::p_timer_start()
{
	if (p_event_list->get_report(p_query_msg_id) != NULL) {
		if (p_event_rate > 0) {
			RCLCPP_DEBUG(logger, "start event timer for %#x with rate %.2f to %s", p_query_msg_id, p_event_rate, requestor.str().c_str());
			p_timer.start();
		}
	}
}

bool InternalEvent::p_can_send(unsigned short id)
{
	std::map<jUnsignedShortInteger, ChronoSysTP>::iterator res = p_last_send.find(id);
	if (res != p_last_send.end()) {
		auto last = std::chrono::steady_clock::now() - res->second;
		int64_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(last).count();
		int64_t interval_ms = (int64_t)(1000.0 / p_event_rate - iop::EventsConfig::RATE_PRECISION);
		return (msecs >= interval_ms);
	}
	return true;
}
