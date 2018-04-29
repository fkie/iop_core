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


#include <iop_events_fkie/InternalEventList.h>
#include <ros/ros.h>
#include <ros/console.h>

using namespace iop;
using namespace urn_jaus_jss_core_Events;


InternalEventList::InternalEventList(JTS::StateMachine *jrHandler)
{
	this->jrHandler = jrHandler;
	if (p_config.get_timeout() > 0) {
		p_timeout_timer = p_nh.createTimer(ros::Duration(1.0), &InternalEventList::p_timeout, this);
	}
}

InternalEventList::~InternalEventList()
{
	if (p_timeout_timer.isValid()) {
		p_timeout_timer.stop();
	}
	p_registered_reports.clear();
	p_events.clear();
}

void InternalEventList::p_timeout(const ros::TimerEvent& event)
{
	lock_type lock(p_mutex);
	ros::Time now = ros::Time::now();
	std::map<jUnsignedByte, boost::shared_ptr<iop::InternalEvent> >::iterator it;
	for (it = p_events.begin(); it != p_events.end(); ++it) {
		if (now - it->second->get_last_update_time() > ros::Duration(p_config.get_timeout() * 60.0)) {
			ROS_WARN_NAMED("Events", "event %d with query %#x to %s timed out, remove",
					it->second->get_event_id(), it->second->get_query_msg_id(), it->second->requestor.str().c_str());
			p_events.erase(it);
			return;
		}
	}
}

void InternalEventList::register_query(jUnsignedShortInteger query_msg_id, bool supports_on_change, bool supports_periodic)
{
	lock_type lock(p_mutex);
	for (unsigned int i = 0; i < p_registered_reports.size(); i++) {
		if (p_registered_reports[i].query_msg_id == query_msg_id) {
			ROS_WARN_NAMED("Events", "query id %d already registered, skip registration", query_msg_id);
			return;
		}
	}
	ROS_INFO_NAMED("Events", "register query id %d", query_msg_id);
	RegisteredReports rep(query_msg_id, supports_on_change, supports_periodic);
	p_registered_reports.push_back(rep);
}

bool InternalEventList::set_report(jUnsignedShortInteger query_msg_id, JTS::Message *report)
{
	lock_type lock(p_mutex);
	for (unsigned int i = 0; i < p_registered_reports.size(); i++) {
		if (p_registered_reports[i].query_msg_id == query_msg_id) {
			p_registered_reports[i].report = report;
			// update current events
			std::map<jUnsignedByte, boost::shared_ptr<iop::InternalEvent> >::iterator it;
			for (it = p_events.begin(); it != p_events.end(); ++it) {
				if (it->second->get_query_msg_id() == query_msg_id) {
					it->second->new_report_available(report);
				}
			}
			return true;
		}
	}
	ROS_WARN_NAMED("Events", "set report for an unregistered query id %#x failed", query_msg_id);
	return false;
}

JTS::Message *InternalEventList::get_report(jUnsignedShortInteger query_msg_id)
{
	lock_type lock(p_mutex);
	for (unsigned int i = 0; i < p_registered_reports.size(); i++) {
		if (p_registered_reports[i].query_msg_id == query_msg_id) {
			return p_registered_reports[i].report;
		}
	}
	return NULL;
}

bool InternalEventList::supports_message(jUnsignedShortInteger query_msg_id)
{
	lock_type lock(p_mutex);
	for (unsigned int i = 0; i < p_registered_reports.size(); i++) {
		if (p_registered_reports[i].query_msg_id == query_msg_id) {
			return true;
		}
	}
	return false;
}

bool InternalEventList::supports_on_change(jUnsignedShortInteger query_msg_id)
{
	lock_type lock(p_mutex);
	for (unsigned int i = 0; i < p_registered_reports.size(); i++) {
		if (p_registered_reports[i].query_msg_id == query_msg_id) {
			return p_registered_reports[i].supports_on_change;
		}
	}
	return false;
}

bool InternalEventList::supports_periodic(jUnsignedShortInteger query_msg_id)
{
	lock_type lock(p_mutex);
	for (unsigned int i = 0; i < p_registered_reports.size(); i++) {
		if (p_registered_reports[i].query_msg_id == query_msg_id) {
			return p_registered_reports[i].supports_periodic;
		}
	}
	return false;
}

bool InternalEventList::supports_rate(jUnsignedShortInteger query_msg_id, double rate)
{
	lock_type lock(p_mutex);
	for (unsigned int i = 0; i < p_registered_reports.size(); i++) {
		if (p_registered_reports[i].query_msg_id == query_msg_id) {
			return p_registered_reports[i].supports_rate(rate);
		}
	}
	return false;
}

void InternalEventList::get_queries(jUnsignedShortInteger query_msg_id, std::map<jUnsignedByte, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage>& result)
{
	lock_type lock(p_mutex);
	std::map<jUnsignedByte, boost::shared_ptr<iop::InternalEvent> >::iterator it;
	for (it = p_events.begin(); it != p_events.end(); ++it) {
		if (it->second->get_query_msg_id() == query_msg_id) {
			result[it->first] = it->second->get_query();
		}
	}
}

void InternalEventList::send_report(jUnsignedByte event_id, JTS::Message &report, unsigned short id)
{
	lock_type lock(p_mutex);
	std::map<jUnsignedByte, boost::shared_ptr<iop::InternalEvent> >::iterator it;
	for (it = p_events.begin(); it != p_events.end(); ++it) {
		if (it->first == event_id) {
			it->second->send_report(report, id);
		}
	}
}

void InternalEventList::get_events_by_query(jUnsignedShortInteger query_msg_id, std::vector<boost::shared_ptr<iop::InternalEvent> >& result)
{
	lock_type lock(p_mutex);
	std::map<jUnsignedByte, boost::shared_ptr<iop::InternalEvent> >::iterator it;
	for (it = p_events.begin(); it != p_events.end(); ++it) {
		if (it->second->get_query_msg_id() == query_msg_id) {
			result.push_back(it->second);
		}
	}
}

void InternalEventList::get_events_by_type(jUnsignedByte event_type, std::vector<boost::shared_ptr<iop::InternalEvent> >& result)
{
	lock_type lock(p_mutex);
	std::map<jUnsignedByte, boost::shared_ptr<iop::InternalEvent> >::iterator it;
	for (it = p_events.begin(); it != p_events.end(); ++it) {
		if (it->second->get_event_type() == event_type) {
			result.push_back(it->second);
		}
	}
}

void InternalEventList::get_events_by_id(jUnsignedByte event_id, std::vector<boost::shared_ptr<iop::InternalEvent> >& result)
{
	lock_type lock(p_mutex);
	std::map<jUnsignedByte, boost::shared_ptr<iop::InternalEvent> >::iterator it;
	for (it = p_events.begin(); it != p_events.end(); ++it) {
		if (it->second->get_event_id() == event_id) {
			result.push_back(it->second);
		}
	}
}

void InternalEventList::get_events_all(std::vector<boost::shared_ptr<iop::InternalEvent> >& result)
{
	lock_type lock(p_mutex);
	std::map<jUnsignedByte, boost::shared_ptr<iop::InternalEvent> >::iterator it;
	for (it = p_events.begin(); it != p_events.end(); ++it) {
		result.push_back(it->second);
	}
}


EventsConfig& InternalEventList::get_config()
{
	return p_config;
}

boost::shared_ptr<iop::InternalEvent> InternalEventList::create_event(CreateEvent msg, JausAddress requestor)
{
	CreateEvent::Body::CreateEventRec *erec = msg.getBody()->getCreateEventRec();
	jUnsignedShortInteger query_msg_id = message_id_from_data(erec->getQueryMessage()->getData());
	jUnsignedByte request_id = erec->getRequestID();
	jUnsignedByte event_type = erec->getEventType();
	double event_rate = erec->getRequestedPeriodicRate();
	jUnsignedByte current_event_id = get_event_id(query_msg_id, requestor);
	if (current_event_id != 255) {
		ROS_DEBUG_NAMED("Events", "create event for query_id: %#x, request_id: %d, event type: %s, rate: %f, sender %s alredy registered -> update...",
				query_msg_id, request_id, (event_type == 0) ? "Periodic" : "Every change", event_rate, requestor.str().c_str());
		CreateEvent::Body::CreateEventRec::QueryMessage *query_msg = erec->getQueryMessage();
		return p_update_event(current_event_id, *query_msg, query_msg_id, requestor, request_id, event_type, event_rate);
	} else {
		lock_type lock(p_mutex);
		jUnsignedByte event_id = get_free_event_id();
		if (event_id < 255) {
			ROS_DEBUG_NAMED("Events", "create event for query_id: %#x, request_id: %d, event type: %s, rate: %f, sender %s -> new event id: %d",
					query_msg_id, request_id, (event_type == 0) ? "Periodic" : "Every change", event_rate, requestor.str().c_str(), (int)event_id);
			CreateEvent::Body::CreateEventRec::QueryMessage *query_msg = erec->getQueryMessage();
			boost::shared_ptr<iop::InternalEvent> event(boost::make_shared<iop::InternalEvent>(this, event_id, request_id, query_msg_id, event_type, event_rate, *query_msg, requestor));
			if (event->get_error_code() == 0) {
				p_events[event_id] = event;
				event->new_report_available(get_report(query_msg_id), false);
			}
			return event;
//
//			p_events.insert(std::map<jUnsignedByte, boost::shared_ptr<iop::InternalEvent> >::value_type(event_id, boost::make_shared<iop::InternalEvent>(*this, event_id, request_id, query_msg_id, event_type, event_rate, *query_msg, requestor)));
//			return p_events.find(event_id)->second;
		} else {
			ROS_WARN_NAMED("Events", "create event for query_id: %#x, request_id: %d, event type: %s, rate: %f, sender %s failed, maximum registered events reached!",
					query_msg_id, request_id, (event_type == 0) ? "Periodic" : "Every change", event_rate, requestor.str().c_str());
			boost::shared_ptr<iop::InternalEvent> event(boost::make_shared<iop::InternalEvent>(this, request_id, query_msg_id, event_type, event_rate));
			event->set_error(4, "maximum registered events reached");
			return event;
		}
	}
	boost::shared_ptr<iop::InternalEvent> event(boost::make_shared<iop::InternalEvent>(this, request_id, query_msg_id, event_type, event_rate));
	event->set_error(4, "unknown error, should not happen");
	return event;
}

boost::shared_ptr<iop::InternalEvent> InternalEventList::update_event(UpdateEvent msg, JausAddress requestor)
{
	UpdateEvent::Body::UpdateEventRec *urec = msg.getBody()->getUpdateEventRec();
	jUnsignedShortInteger query_msg_id = message_id_from_data(urec->getQueryMessage()->getData());
	jUnsignedByte event_id = urec->getEventID();
	jUnsignedByte request_id = urec->getRequestID();
	jUnsignedByte event_type = urec->getEventType();
	double event_rate = urec->getRequestedPeriodicRate();
	CreateEvent::Body::CreateEventRec::QueryMessage query_msg;
	query_msg.set(urec->getQueryMessage()->getLength(), urec->getQueryMessage()->getData());
	return p_update_event(event_id, query_msg, query_msg_id, requestor, request_id, event_type, event_rate);
}

boost::shared_ptr<iop::InternalEvent> InternalEventList::p_update_event(jUnsignedByte event_id, CreateEvent::Body::CreateEventRec::QueryMessage query_msg, jUnsignedShortInteger query_msg_id, JausAddress requestor, jUnsignedByte request_id, jUnsignedByte event_type, double event_rate)
{
	lock_type lock(p_mutex);
	std::map<jUnsignedByte, boost::shared_ptr<iop::InternalEvent> >::iterator it;
	for (it = p_events.begin(); it != p_events.end(); ++it) {
		if (it->first == event_id) {
			ROS_DEBUG_NAMED("Events", "update event %d with query_id: %#x, request_id: %d, event type: %s, rate: %f, sender %s",
					(int)event_id, query_msg_id, request_id, (event_type == 0) ? "Periodic" : "Every change", event_rate, requestor.str().c_str());
			boost::shared_ptr<iop::InternalEvent> test_event(boost::make_shared<iop::InternalEvent>(this, request_id, query_msg_id, event_type, event_rate));
			if (test_event->get_error_code() == 0) {
				it->second->update(event_id, query_msg, query_msg_id, requestor, request_id, event_type, event_rate);
				return it->second;
			} else {
				ROS_DEBUG_NAMED("Events", "update event %d failed: error %d: %s",
						(int)event_id, (int)test_event->get_error_code(), test_event->get_error_msg().c_str());
				return test_event;
			}
		}
	}
	boost::shared_ptr<iop::InternalEvent> event(boost::make_shared<iop::InternalEvent>(this, request_id, query_msg_id, event_type, event_rate));
	event->set_error(6, "Invalid event ID for update event request");
	return event;

}

bool InternalEventList::cancel_event(urn_jaus_jss_core_Events::CancelEvent msg, JausAddress requestor)
{
	lock_type lock(p_mutex);
	jUnsignedByte event_id = msg.getBody()->getCancelEventRec()->getEventID();
	std::map<jUnsignedByte, boost::shared_ptr<iop::InternalEvent> >::iterator res = p_events.find(event_id);
	if (res != p_events.end()) {
		if (res->second->requestor == requestor) {
			ROS_DEBUG_NAMED("Events", "remove event with id %d for query message %#x, requested from %s",
					(int)event_id, res->second->get_query_msg_id(), requestor.str().c_str());
			p_events.erase(res);
			return true;
		} else {
			ROS_WARN_NAMED("Events", "the requestor %s to cancel event %d is not the creator %s, ignore!",
					requestor.str().c_str(), (int)event_id, res->second->requestor.str().c_str());
		}
	} else {
		ROS_DEBUG_NAMED("Events", "no event with id %d to cancel found, requested from %s -> ignore!",
				(int)event_id, requestor.str().c_str());
	}
	return false;
}

jUnsignedShortInteger InternalEventList::message_id_from_data(const unsigned char *data)
{
	jUnsignedShortInteger result = 0;
	try {
		memcpy(&result, data, sizeof(jUnsignedShortInteger));
		result = JSIDL_v_1_0::correctEndianness(result);
	} catch (...) {
	}
	return result;
}

jUnsignedByte InternalEventList::get_free_event_id()
{
	lock_type lock(p_mutex);
	jUnsignedByte result = 0;
	while (result < 255) {
		if (p_events.find(result) != p_events.end()) {
			result++;
		} else {
			break;
		}
	}
	return result;
}

jUnsignedByte InternalEventList::get_event_id(jUnsignedShortInteger query_msg_id, JausAddress requestor)
{
	lock_type lock(p_mutex);
	for (std::map<jUnsignedByte, boost::shared_ptr<iop::InternalEvent> >::iterator it=p_events.begin(); it!=p_events.end(); it++) {
		if (it->second->get_query_msg_id() == query_msg_id && it->second->requestor == requestor) {
			return it->first;
		}
	}
	return 255;
}


bool InternalEventList::has_event(jUnsignedShortInteger query_msg_id, JausAddress requestor)
{
	return (get_event_id(query_msg_id, requestor) != 255);
}

InternalEventList::RegisteredReports::RegisteredReports()
{
	query_msg_id = 65535;
	supports_on_change = false;
	supports_periodic = false;
	min_rate = EventsConfig::MINIMUM_RATE;
	max_rate = EventsConfig::MAXIMUM_RATE;
	report = NULL;
}

InternalEventList::RegisteredReports::RegisteredReports(jUnsignedShortInteger query_msg_id, bool supports_on_change, bool supports_periodic)
{
	this->query_msg_id = query_msg_id;
	this->supports_on_change = supports_on_change;
	this->supports_periodic = supports_periodic;
	min_rate = EventsConfig::MINIMUM_RATE;
	max_rate = EventsConfig::MAXIMUM_RATE;
	report = NULL;
}

bool InternalEventList::RegisteredReports::operator==(InternalEventList::RegisteredReports &value)
{
	return query_msg_id == value.query_msg_id;
}

bool InternalEventList::RegisteredReports::operator!=(InternalEventList::RegisteredReports &value)
{
	return query_msg_id != value.query_msg_id;
}

bool InternalEventList::RegisteredReports::supports_rate(double rate)
{
	return (rate >= min_rate && rate <= max_rate);
}
