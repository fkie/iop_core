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


#ifndef IOPINTERNALEVENTLIST_H
#define IOPINTERNALEVENTLIST_H

#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_core_Events/Messages/MessageSet.h"
#include "urn_jaus_jss_core_Events/InternalEvents/InternalEventsSet.h"
#include "InternalEvent.h"

#include <boost/thread/recursive_mutex.hpp>
#include <ros/ros.h>
#include <string>

#include "EventsConfig.h"

namespace iop
{

class InternalEventList {
public:
	InternalEventList(JTS::StateMachine *jrHandler);
	~InternalEventList();

	void register_query(jUnsignedShortInteger query_msg_id, bool supports_on_change=true, bool supports_periodic=true);
	/** register_query() should be called before use this method. Otherwise it will return false. */
	bool set_report(jUnsignedShortInteger query_msg_id, JTS::Message *report);
	JTS::Message *get_report(jUnsignedShortInteger query_msg_id);
	bool supports_message(jUnsignedShortInteger query_msg_id);
	bool supports_on_change(jUnsignedShortInteger query_msg_id);
	bool supports_periodic(jUnsignedShortInteger query_msg_id);
	bool supports_rate(jUnsignedShortInteger query_msg_id, double rate);

	EventsConfig& get_config();
	JTS::StateMachine *get_jr_handler() { return jrHandler; }

	/** ========= methods to apply filter from queries ======= **/
	/** Fills the map with event id's and query messages, which are send by requestor. */
	void get_queries(jUnsignedShortInteger query_msg_id, std::map<jUnsignedByte, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage>& result);
	/** If you use filter setted by query message, use send_report() to send filtered reports. */
	void send_report(jUnsignedByte event_id, JTS::Message &report, unsigned short id=0);

	/** ========= methods to get events ======= **/
	void get_events_by_query(jUnsignedShortInteger query_msg_id, std::vector<boost::shared_ptr<iop::InternalEvent> >& result);
	void get_events_by_type(jUnsignedByte event_type, std::vector<boost::shared_ptr<iop::InternalEvent> >& result);
	void get_events_by_id(jUnsignedByte event_id, std::vector<boost::shared_ptr<iop::InternalEvent> >& result);
	void get_events_all(std::vector<boost::shared_ptr<iop::InternalEvent> > &result);

	/** ========= methods to manage events ======= **/
	boost::shared_ptr<iop::InternalEvent> create_event(urn_jaus_jss_core_Events::CreateEvent msg, JausAddress requestor);
	boost::shared_ptr<iop::InternalEvent> update_event(urn_jaus_jss_core_Events::UpdateEvent msg, JausAddress requestor);
	bool cancel_event(urn_jaus_jss_core_Events::CancelEvent msg, JausAddress requestor);

	/** ========= helper methods ======= **/
	jUnsignedShortInteger message_id_from_data(const unsigned char *data);
	jUnsignedByte get_free_event_id();
	jUnsignedByte get_event_id(jUnsignedShortInteger query_msg_id, JausAddress requestor);
	bool has_event(jUnsignedShortInteger query_msg_id, JausAddress requestor);

protected:
	class RegisteredReports {
	public:
		jUnsignedShortInteger query_msg_id;
		bool supports_on_change;
		bool supports_periodic;
		double min_rate;
		double max_rate;
		JTS::Message *report;
		RegisteredReports();
		RegisteredReports(jUnsignedShortInteger query_msg_id, bool supports_on_change=true, bool supports_periodic=true);
		bool operator==(RegisteredReports &value);
		bool operator!=(RegisteredReports &value);
		bool supports_rate(double rate);
	};
	typedef boost::recursive_mutex mutex_type;
	typedef boost::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;

	ros::NodeHandle p_nh;
	ros::Timer p_timeout_timer;
	EventsConfig p_config;
	JTS::StateMachine *jrHandler;
	std::vector<RegisteredReports> p_registered_reports;
	std::map<jUnsignedByte, boost::shared_ptr<iop::InternalEvent> > p_events;  //event id, current active events

	boost::shared_ptr<iop::InternalEvent> p_update_event(jUnsignedByte event_id, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage query_msg, jUnsignedShortInteger query_msg_id, JausAddress requestor, jUnsignedByte request_id, jUnsignedByte event_type, double event_rate);
	void p_timeout(const ros::TimerEvent& event);
};

};

#endif
