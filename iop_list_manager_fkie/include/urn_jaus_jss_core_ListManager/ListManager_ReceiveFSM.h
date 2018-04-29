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


#ifndef LISTMANAGER_RECEIVEFSM_H
#define LISTMANAGER_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_core_ListManager/Messages/MessageSet.h"
#include "urn_jaus_jss_core_ListManager/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_core_Management/Management_ReceiveFSM.h"

#include <ros/ros.h>
#include <iop_list_manager_fkie/InternalElementList.h>

#include "ListManager_ReceiveFSM_sm.h"

namespace urn_jaus_jss_core_ListManager
{

class DllExport ListManager_ReceiveFSM : public JTS::StateMachine
{
public:
	ListManager_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM);
	virtual ~ListManager_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void deleteElementAction(DeleteElement msg);
	virtual void sendConfirmElementRequestAction(DeleteElement msg, Receive::Body::ReceiveRec transportData);
	virtual void sendRejectElementRequestAction(DeleteElement msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportElementAction(QueryElement msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportElementCountAction(QueryElementCount msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportElementListAction(QueryElementList msg, Receive::Body::ReceiveRec transportData);
	virtual void setElementAction(SetElement msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	virtual bool elementExists(DeleteElement msg);
	virtual bool elementExists(QueryElement msg);
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);

	iop::InternalElementList& list_manager();

	ListManager_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM;

	iop::InternalElementList p_list;
};

};

#endif // LISTMANAGER_RECEIVEFSM_H
