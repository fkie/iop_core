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


#ifndef IOPINTERNALEVENT_H
#define IOPINTERNALEVENT_H

#include "Transport/JausTransport.h"
#include "urn_jaus_jss_core_Events/Messages/MessageSet.h"
#include "urn_jaus_jss_core_Events/InternalEvents/InternalEventsSet.h"

#include <ros/ros.h>
#include <string>

#include "EventsConfig.h"

namespace iop
{

class InternalEventList;

class InternalEvent {

public:
	urn_jaus_jss_core_Events::Event event;
	jUnsignedByte seq_nr;
	JausAddress requestor;
	InternalEvent(InternalEventList* event_list);
	InternalEvent(InternalEventList* event_list, jUnsignedByte request_id, jUnsignedShortInteger query_msg_id, jUnsignedByte event_type=0, double event_rate=0.0);
	InternalEvent(InternalEventList* event_list, jUnsignedByte event_id, jUnsignedByte request_id, jUnsignedShortInteger query_msg_id, jUnsignedByte event_type, double event_rate, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage query_msg, JausAddress requestor);
	~InternalEvent();
	bool operator==(InternalEvent &value);
	bool operator!=(InternalEvent &value);

	jUnsignedByte get_request_id() { return p_request_id; }
	jUnsignedByte get_event_id() { return p_event_id; }
	/** event_type: 0: Periodic, 1: every change **/
	jUnsignedByte get_event_type() { return p_event_type; }
	double get_event_rate() { return p_event_rate; }
	jUnsignedShortInteger get_query_msg_id() { return p_query_msg_id; }
	urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage& get_query_msg() { return p_query_msg; }

	/** This method is called if the send interval is handled by event service and the report is only updated in some cases.
	 * Use send_report() if the send interval and report content is handled the the caller class. */
	void new_report_available(JTS::Message *report, bool send_if_possible=true);
	/** sends report, if one is availabe after creation */
	void send_last_report();
	urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage &get_query();
	/** If you use filter setted by query message, use send_report() to send filtered reports. */
	void send_report(JTS::Message &report, unsigned short id=0);
	void update(jUnsignedByte event_id, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage query_msg, jUnsignedShortInteger query_msg_id, JausAddress requestor, jUnsignedByte request_id, jUnsignedByte event_type, double event_rate);

	/** Returns true if it was initialized. */
	bool is_valid() { return p_initialized; }
	/**
		0: no error
		1: Periodic events not supported
		2: Change based events not supported
		3: Connection refused
		4: Invalid event setup
		5: Message not supported
		6: Invalid event ID for update event request
	 */
	jUnsignedByte get_error_code() { return p_error_code; }
	/** Returns message for current error. */
	std::string get_error_msg() { return p_error_msg; }
	void set_error(jUnsignedByte code, std::string msg="");

	ros::Time get_last_update_time() { return p_last_update; }

protected:
	InternalEventList *p_event_list;
	ros::NodeHandle p_nh;
	ros::Timer p_timeout_timer;
	ros::Time p_last_update;
	urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage p_query_msg;
	jUnsignedShortInteger p_query_msg_id;
	jUnsignedByte p_request_id;
	jUnsignedByte p_event_id;
	jUnsignedByte p_event_type;
	double p_event_rate;
	bool p_supports_on_change;
	bool p_supports_periodic;
	jUnsignedByte p_error_code;
	std::string p_error_msg;
	bool p_initialized;
	JTS::Message *p_last_report;
	std::map<jUnsignedShortInteger, ros::Time> p_last_send;  // some sensor id of a report, last send time

	bool p_is_event_supported(jUnsignedShortInteger query_msg_id, jUnsignedByte p_event_type, double p_event_rate);
	void p_send_as_event(JTS::Message &report, JausAddress &address);

	void timeout(const ros::TimerEvent& event);
	void p_timer_stop();
	void p_timer_start();

	bool p_can_send(unsigned short id=0);
private:
	InternalEvent(InternalEvent const& from);
	const InternalEvent& operator=(const InternalEvent& from);

};

};

#endif
