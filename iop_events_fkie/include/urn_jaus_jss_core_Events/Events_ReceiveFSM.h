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


#ifndef EVENTS_RECEIVEFSM_H
#define EVENTS_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_core_Events/Messages/MessageSet.h"
#include "urn_jaus_jss_core_Events/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"

#include "iop_events_fkie/InternalEventList.h"
#include "Events_ReceiveFSM_sm.h"

#include <ros/ros.h>

namespace urn_jaus_jss_core_Events
{

class DllExport Events_ReceiveFSM : public JTS::StateMachine
{
public:
	Events_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~Events_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void cancelEventAction(CancelEvent msg, Receive::Body::ReceiveRec transportData);
	virtual void createCommandEventAction(CreateCommandEvent msg, Receive::Body::ReceiveRec transportData);
	virtual void createEventAction(CreateEvent msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportEventTimeoutAction(QueryEventTimeout msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportEventsAction(QueryEvents msg, Receive::Body::ReceiveRec transportData);
	virtual void updateEventAction(UpdateEvent msg, Receive::Body::ReceiveRec transportData);

	/// User Methods
//	void eventOccurred(void* arg);
	/** Register/update the supported event. The caller has to store the report, because only the given reference to the
	 * report will be stored and used to send the report to a client, which create an event.
	 * If on change is supported you have to call this method each time the report was changed.
	 * **/
//	void set_event_report(jUnsignedShortInteger query_msg_id, JTS::Message &report, bool on_change_supported=true);
	iop::InternalEventList& get_event_handler() { return p_event_list; }

	Events_ReceiveFSMContext *context;

protected:
    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	iop::InternalEventList p_event_list;

};

};

#endif // EVENTS_RECEIVEFSM_H
