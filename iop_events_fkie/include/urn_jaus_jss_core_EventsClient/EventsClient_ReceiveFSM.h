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


#ifndef EVENTSCLIENT_RECEIVEFSM_H
#define EVENTSCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_core_EventsClient/Messages/MessageSet.h"
#include "urn_jaus_jss_core_EventsClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <ros/ros.h>
#include <iop_events_fkie/InternalEventClient.h>
#include <iop_events_fkie/EventHandlerInterface.h>
#include "EventsClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_core_EventsClient
{

class DllExport EventsClient_ReceiveFSM : public JTS::StateMachine
{
public:
	EventsClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~EventsClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleCommandEventAction(CommandEvent msg, Receive::Body::ReceiveRec transportData);
	virtual void handleConfirmEventRequestAction(ConfirmEventRequest msg, Receive::Body::ReceiveRec transportData);
	virtual void handleEventAction(Event msg, Receive::Body::ReceiveRec transportData);
	virtual void handleRejectEventRequestAction(RejectEventRequest msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportEventTimeoutAction(ReportEventTimeout msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportEventsAction(ReportEvents msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods



	EventsClient_ReceiveFSMContext *context;
	/** Create an event on given remote JAUS service. By default it is a periodic event.
	 *
	 * rate:
	 * MINIMUM_RATE = 0.1
	 * MAXIMUM_RATE = 25.0
	 *
	 * event_type:
	 * 0 = Periodic, on valid rate
	 * 1 = Every Change, if rate is less then 0.1 or more then 25 */
	void create_event(iop::EventHandlerInterface &handler, JausAddress address, JTS::Message &query_msg, double rate=1.0);
	void cancel_event(iop::EventHandlerInterface &handler, JausAddress address, JTS::Message &query_msg);
	/** You can register a handler to be informed about the creation/cancelation status of your events. To register the handler you
	 * have to define a function in your class:
	 * void my_handler(JausAddress &addr, jUnsignedShortInteger query_msg_id, bool accepted, jByte event_id, jUnsignedByte reject_code) {}
	 * and then register:
	 * pEventsClientService->set_events_reply_handler(&MyClass::my_handler, this);
	 *
	 * reject_code:
	 * 1: Periodic events not supported
	 * 2: Change based events not supported
	 * 3: Connection refused
	 * 4: Invalid event setup
	 * 5: Message not supported
	 * 6: Invalid event ID for update event request */
	template<class T>
	void set_events_reply_handler(void(T::*handler)(JausAddress &, jUnsignedShortInteger query_msg_id, bool accepted, jByte event_id, jUnsignedByte reject_code), T*obj) {
		p_class_events_reply_callback = boost::bind(handler, obj, _1, _2, _3, _4, _5);
	}

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	std::vector<iop::InternalEventClient *> p_events;
	jUnsignedByte p_request_id_idx;
	typedef boost::recursive_mutex mutex_type;
	typedef boost::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;
	boost::function<void (JausAddress &, jUnsignedShortInteger query_msg_id, bool accepted, jByte event_id, jUnsignedByte reject_code)> p_class_events_reply_callback;

	iop::InternalEventClient* p_get_event(JausAddress address, jUnsignedShortInteger query_msg_id);
};

};

#endif // EVENTSCLIENT_RECEIVEFSM_H
