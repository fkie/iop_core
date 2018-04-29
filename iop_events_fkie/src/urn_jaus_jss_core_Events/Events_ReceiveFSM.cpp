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

#include "iop_events_fkie/InternalEvent.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"

#include <ros/console.h>


using namespace JTS;

namespace urn_jaus_jss_core_Events
{

Events_ReceiveFSM::Events_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: p_event_list(this)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new Events_ReceiveFSMContext(*this);
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
}

Events_ReceiveFSM::~Events_ReceiveFSM()
{
	delete context;
}

void Events_ReceiveFSM::setupNotifications()
{
	pTransport_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_Events_ReceiveFSM_Receiving_Ready", "Transport_ReceiveFSM");
	registerNotification("Receiving_Ready", pTransport_ReceiveFSM->getHandler(), "InternalStateChange_To_Transport_ReceiveFSM_Receiving", "Events_ReceiveFSM");
	registerNotification("Receiving", pTransport_ReceiveFSM->getHandler(), "InternalStateChange_To_Transport_ReceiveFSM_Receiving", "Events_ReceiveFSM");
	ROS_DEBUG_NAMED("Events", "Events service created");
}

void Events_ReceiveFSM::cancelEventAction(CancelEvent msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress requestor = transportData.getAddress();
	jUnsignedByte request_id = msg.getBody()->getCancelEventRec()->getRequestID();
	jUnsignedByte event_id = msg.getBody()->getCancelEventRec()->getEventID();
	if (p_event_list.cancel_event(msg, requestor)) {
		ROS_DEBUG_NAMED("Events", "cancelEventAction, send ConfirmEvent for event id: %d", (int)event_id);
		ConfirmEventRequest response;
		response.getBody()->getConfirmEventRequestRec()->setEventID(event_id);
		response.getBody()->getConfirmEventRequestRec()->setRequestID(request_id);
		response.getBody()->getConfirmEventRequestRec()->setConfirmedPeriodicRate(0);
		sendJausMessage( response, requestor );
	} else {
		RejectEventRequest revent;
		RejectEventRequest::Body::RejectEventRequestRec *rvent_rec = revent.getBody()->getRejectEventRequestRec();
		rvent_rec->setRequestID(request_id);
		rvent_rec->setResponseCode(6);
	//	rvent_rec->setErrorMessage("error, invalid event ID for cancel event");
		ROS_DEBUG_NAMED("Events", "cancelEventAction send RejectEventRequest for event id %d with request id %d to %s",
				(int)event_id, (int)request_id, requestor.str().c_str());
		sendJausMessage( revent, requestor );
	}
}

void Events_ReceiveFSM::createCommandEventAction(CreateCommandEvent msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress requestor = transportData.getAddress();
	RejectEventRequest revent;
	RejectEventRequest::Body::RejectEventRequestRec *rvent_rec = revent.getBody()->getRejectEventRequestRec();
	jUnsignedByte request_id = msg.getBody()->getCreateEventRec()->getRequestID();
	rvent_rec->setRequestID(request_id);
	rvent_rec->setResponseCode(6);
	rvent_rec->setErrorMessage("command events are not implemented in FKIE ROS/IOP Bridge");
	ROS_WARN_NAMED("Events", "Create Command Event not implemented!");
	ROS_DEBUG_NAMED("Events", "createCommandEventAction send RejectEventRequest for request id %d to %s",
			(int)request_id, requestor.str().c_str());
	sendJausMessage( revent, requestor );
}

void Events_ReceiveFSM::createEventAction(CreateEvent msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress requestor = transportData.getAddress();
	boost::shared_ptr<iop::InternalEvent> event = p_event_list.create_event(msg, requestor);
	if (event->is_valid()) {
		// send confirm message
		ConfirmEventRequest response;
		response.getBody()->getConfirmEventRequestRec()->setRequestID(event->get_request_id());
		response.getBody()->getConfirmEventRequestRec()->setEventID(event->get_event_id());
		response.getBody()->getConfirmEventRequestRec()->setConfirmedPeriodicRate(event->get_event_rate());
		ROS_DEBUG_NAMED("Events", "send confirm create event to %s, request id: %d, query msg id: %#x",
				requestor.str().c_str(), event->get_request_id(), event->get_query_msg_id());
		sendJausMessage( response, requestor );
		event->send_last_report();
	} else {
		// send Reject message
		RejectEventRequest revent;
		RejectEventRequest::Body::RejectEventRequestRec *rvent_rec = revent.getBody()->getRejectEventRequestRec();
		rvent_rec->setRequestID(event->get_request_id());
		rvent_rec->setResponseCode(event->get_error_code());
		std::string rmsg = event->get_error_msg();
		if (!rmsg.empty()) {
			rvent_rec->setErrorMessage(rmsg);
		}
		ROS_DEBUG_NAMED("Events", "send REJECT create event to %s, CreateEvent: %d, rcode: %d, msg: %s",
				requestor.str().c_str(), event->get_request_id(), event->get_error_code(), rmsg.c_str());
		sendJausMessage( revent, requestor );
	}
}

void Events_ReceiveFSM::sendReportEventTimeoutAction(QueryEventTimeout msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress requestor = transportData.getAddress();
	ReportEventTimeout response;
	jUnsignedByte timeout = p_event_list.get_config().get_timeout();
	ROS_DEBUG_NAMED("Events", "send EventTimeout to %s, timeout: %d", requestor.str().c_str(), timeout);
	response.getBody()->getReportTimoutRec()->setTimeout(timeout);
	sendJausMessage( response, requestor );
}

void Events_ReceiveFSM::sendReportEventsAction(QueryEvents msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress requestor = transportData.getAddress();
	ReportEvents response;
	try {
		QueryEvents::Body::QueryEventsVar *evrec = msg.getBody()->getQueryEventsVar();
		jUnsignedByte filter = evrec->getFieldValue();
		std::vector<boost::shared_ptr<iop::InternalEvent> > events;
		// add filter support specified in QueryEvents message
		switch(filter) {
			case 0:
				p_event_list.get_events_by_query(evrec->getMessageIDRec()->getMessageCode(), events);
				break;
			case 1:
				p_event_list.get_events_by_type(evrec->getEventTypeRec()->getEventType(), events);
				break;
			case 2:
				p_event_list.get_events_by_id(evrec->getEventIDRec()->getEventID(), events);
				break;
			case 3:
				p_event_list.get_events_all(events);
				break;
		}
		for (unsigned int i = 0; i < events.size(); i++) {
			ReportEvents::Body::EventList::ReportEventRec event;
			boost::shared_ptr<iop::InternalEvent> &ievent = events[i];
			event.setEventID(ievent->get_event_id());
			event.setEventType(ievent->get_event_type());
			event.getQueryMessage()->set(ievent->get_query_msg().getLength(), ievent->get_query_msg().getData());
			response.getBody()->getEventList()->addElement(event);
		}
	} catch (std::exception &e) {
		ROS_WARN_NAMED("Events", "ERR %s", e.what());
	}
	ROS_DEBUG_NAMED("Events", "sendReportEventsAction to %s, event count: %d",
			requestor.str().c_str(), response.getBody()->getEventList()->getNumberOfElements());
	sendJausMessage( response, requestor );
}

void Events_ReceiveFSM::updateEventAction(UpdateEvent msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress requestor = transportData.getAddress();
	boost::shared_ptr<iop::InternalEvent> event = p_event_list.update_event(msg, requestor);
	int event_id = (int)msg.getBody()->getUpdateEventRec()->getEventID();
	if (event->is_valid()) {
		// send confirm message
		ConfirmEventRequest response;
		response.getBody()->getConfirmEventRequestRec()->setRequestID(event->get_request_id());
		response.getBody()->getConfirmEventRequestRec()->setEventID(event->get_event_id());
		response.getBody()->getConfirmEventRequestRec()->setConfirmedPeriodicRate(event->get_event_rate());
		ROS_DEBUG_NAMED("Events", "send confirm update event %d to %s, request id: %d, query msg id: %#x",
				event_id, requestor.str().c_str(), event->get_request_id(), event->get_query_msg_id());
		sendJausMessage( response, requestor );
	} else {
		// send Reject message
		RejectEventRequest revent;
		RejectEventRequest::Body::RejectEventRequestRec *rvent_rec = revent.getBody()->getRejectEventRequestRec();
		rvent_rec->setRequestID(event->get_request_id());
		rvent_rec->setResponseCode(event->get_error_code());
		std::string rmsg = event->get_error_msg();
		if (!rmsg.empty()) {
			rvent_rec->setErrorMessage(rmsg);
		}
		ROS_DEBUG_NAMED("Events", "send REJECT update event %d to %s, UpdateEvent: %d, rcode: %d, msg: %s",
				event_id, requestor.str().c_str(), event->get_request_id(), event->get_error_code(), rmsg.c_str());
		sendJausMessage( revent, requestor );
	}
}


};
