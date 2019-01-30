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


#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"

#include <ros/console.h>


using namespace JTS;

namespace urn_jaus_jss_core_EventsClient
{



EventsClient_ReceiveFSM::EventsClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new EventsClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	p_request_id_idx = 0;
}



EventsClient_ReceiveFSM::~EventsClient_ReceiveFSM()
{
	delete context;
	//lock_type lock(p_mutex);
	std::vector<iop::InternalEventClient *>::iterator it;
	for (it = p_events.begin(); it != p_events.end(); ++it) {
		delete (*it);
	}
	p_events.clear();
}

void EventsClient_ReceiveFSM::setupNotifications()
{
	pTransport_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving_Ready", "Transport_ReceiveFSM");
	registerNotification("Receiving_Ready", pTransport_ReceiveFSM->getHandler(), "InternalStateChange_To_Transport_ReceiveFSM_Receiving", "EventsClient_ReceiveFSM");
	registerNotification("Receiving", pTransport_ReceiveFSM->getHandler(), "InternalStateChange_To_Transport_ReceiveFSM_Receiving", "EventsClient_ReceiveFSM");

}

void EventsClient_ReceiveFSM::handleCommandEventAction(CommandEvent msg, Receive::Body::ReceiveRec transportData)
{
	//TODO
}

void EventsClient_ReceiveFSM::handleConfirmEventRequestAction(ConfirmEventRequest msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	jUnsignedByte request_id = msg.getBody()->getConfirmEventRequestRec()->getRequestID();
	ROS_DEBUG_NAMED("EventsClient", "Confirmed for request_id: %d to %s, rate: %.2f", request_id,
			sender.str().c_str(), msg.getBody()->getConfirmEventRequestRec()->getConfirmedPeriodicRate());
	lock_type lock(p_mutex);
	std::vector<iop::InternalEventClient *>::iterator it;
	for (it = p_events.begin(); it != p_events.end(); ++it) {
		if ((*it)->handle_confirm(msg, sender)) {
			if (p_class_events_reply_callback) {
				p_class_events_reply_callback(sender, (*it)->get_query_msg_id(), true, msg.getBody()->getConfirmEventRequestRec()->getEventID(), 0);
			}
			if ((*it)->is_canceld()) {
				delete (*it);
				p_events.erase(it);
				break;
			}
		}
	}
}

void EventsClient_ReceiveFSM::handleEventAction(Event msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	jUnsignedByte event_id = msg.getBody()->getEventRec()->getEventID();
	ROS_DEBUG_NAMED("EventsClient.Event", "event %d from %s received, seqnr: %d", (int)event_id,
			sender.str().c_str(), msg.getBody()->getEventRec()->getSequenceNumber());
	lock_type lock(p_mutex);
	std::vector<iop::InternalEventClient *>::iterator it;
	for (it = p_events.begin(); it != p_events.end(); ++it) {
		if ((*it)->handle_event(msg, sender)) {
		}
	}
}

void EventsClient_ReceiveFSM::handleRejectEventRequestAction(RejectEventRequest msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	jUnsignedByte request_id = msg.getBody()->getRejectEventRequestRec()->getRequestID();
	ROS_DEBUG_NAMED("EventsClient", "Rejected for request_id: %d to %s, code: %d", request_id,
			sender.str().c_str(), msg.getBody()->getRejectEventRequestRec()->getResponseCode());
	lock_type lock(p_mutex);
	std::vector<iop::InternalEventClient *>::iterator it;
	for (it = p_events.begin(); it != p_events.end(); ++it) {
		if ((*it)->handle_reject(msg, sender)) {
			if (p_class_events_reply_callback) {
				p_class_events_reply_callback(sender, (*it)->get_query_msg_id(), false, 255, msg.getBody()->getRejectEventRequestRec()->getResponseCode());
			}
			delete (*it);
			p_events.erase(it);
			break;
		}
	}
}

void EventsClient_ReceiveFSM::handleReportEventTimeoutAction(ReportEventTimeout msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress reporter(transportData.getAddress());
	jUnsignedByte rto = msg.getBody()->getReportTimoutRec()->getTimeout();
	ROS_DEBUG_NAMED("EventsClient", "ReportEventTimeout from %s received: %d sec", reporter.str().c_str(), (int)rto);
	lock_type lock(p_mutex);
	std::vector<iop::InternalEventClient *>::iterator it;
	for (it = p_events.begin(); it != p_events.end(); ++it) {
		(*it)->set_timeout(msg, reporter);
	}
}

void EventsClient_ReceiveFSM::handleReportEventsAction(ReportEvents msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void EventsClient_ReceiveFSM::create_event(iop::EventHandlerInterface &handler, JausAddress address, JTS::Message &query_msg, double rate)
{
	lock_type lock(p_mutex);
	iop::InternalEventClient* event = p_get_event(address, query_msg.getID());
	if (event == NULL) {
		jUnsignedByte event_type = 0;
		if (rate < 0.1 || rate > 25.0) {
			event_type = 1;
		}
		iop::InternalEventClient *event = new iop::InternalEventClient(*this, handler, p_request_id_idx, query_msg, address, event_type, rate);
		p_events.push_back(event);
		event = p_get_event(address, query_msg.getID());
		p_request_id_idx++;
	}
	if (event != NULL) {
		event->add_handler(handler);
	}
}

void EventsClient_ReceiveFSM::cancel_event(iop::EventHandlerInterface &handler, JausAddress address, JTS::Message &query_msg)
{
	lock_type lock(p_mutex);
	iop::InternalEventClient* event = p_get_event(address, query_msg.getID());
	if (event != NULL) {
		event->cancel_event(handler);
	}
}

iop::InternalEventClient* EventsClient_ReceiveFSM::p_get_event(JausAddress address, jUnsignedShortInteger query_msg_id)
{
	lock_type lock(p_mutex);
	std::vector<iop::InternalEventClient *>::iterator it;
	for (it = p_events.begin(); it != p_events.end(); ++it) {
		iop::InternalEventClient* result = *it;
		if (result->get_query_msg_id() == query_msg_id
				&& result->get_remote() == address) {
			return result;
		}
	}
	return NULL;
}


};
