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


#ifndef IOPINTERNALEVENTCLIENT_H
#define IOPINTERNALEVENTCLIENT_H

#include "Transport/JausTransport.h"
#include "urn_jaus_jss_core_Events/Messages/MessageSet.h"
#include "urn_jaus_jss_core_Events/InternalEvents/InternalEventsSet.h"

#include <ros/ros.h>
#include <string>

#include <iop_events_fkie/EventHandlerInterface.h>

namespace urn_jaus_jss_core_EventsClient {
class EventsClient_ReceiveFSM;
};

namespace iop
{

class InternalEventClient {

public:
	/** Create an event on given remote JAUS service. By default it is a periodic event.
	 *
	 * rate:
	 * MINIMUM_RATE = 0.1
	 * MAXIMUM_RATE = 25.0
	 *
	 * event_type:
	 * 0 = Periodic
	 * 1 = Every Change */
	InternalEventClient(urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM& parent, iop::EventHandlerInterface& handler, jUnsignedByte request_id, JTS::Message &query_msg, JausAddress address, jUnsignedByte event_type=0, double rate=1.0);
	~InternalEventClient();
//	InternalEventClient(InternalEventClient const& from);
//	const InternalEventClient& operator=(const InternalEventClient& from);
	bool operator==(InternalEventClient &value);
	bool operator!=(InternalEventClient &value);

	void set_timeout(urn_jaus_jss_core_EventsClient::ReportEventTimeout &msg, JausAddress &reporter);
	bool handle_event(urn_jaus_jss_core_EventsClient::Event &msg, JausAddress &reporter);
	bool handle_confirm(urn_jaus_jss_core_EventsClient::ConfirmEventRequest &msg, JausAddress &reporter);
	bool handle_reject(urn_jaus_jss_core_EventsClient::RejectEventRequest &msg, JausAddress &reporter);
	bool is_canceld() { return p_canceled; }

	/** Callbacks for this event. */
	void add_handler(iop::EventHandlerInterface& handler);
	void remove_handler(iop::EventHandlerInterface& handler);
	void cancel_event(iop::EventHandlerInterface &handler);
	jUnsignedShortInteger get_query_msg_id() { return p_query_msg_id; }
	JausAddress get_remote() { return p_remote; }

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

protected:
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM *p_parent;
	ros::NodeHandle p_nh;
	ros::Timer p_timeout_timer;
	int p_timeout;
	JausAddress p_remote;
	jUnsignedShortInteger p_query_msg_id;
	JTS::Message *p_query_msg;
	jUnsignedByte p_request_id;
	jUnsignedByte p_event_id;
	jUnsignedByte p_event_type;
	double p_event_rate;
	jUnsignedByte p_error_code;
	std::string p_error_msg;
	bool p_wait_for_cancel;
	bool p_canceled;

	std::vector<iop::EventHandlerInterface *> p_handler;

	void p_send_cancel_event();
	void p_send_update_event();

	void timeout(const ros::TimerEvent& event);
	void p_timer_stop();
	void p_timer_start();

private:
	InternalEventClient(const InternalEventClient& that);
	const InternalEventClient& operator=(const InternalEventClient& from);
};

};

#endif
