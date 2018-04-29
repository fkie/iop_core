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

#ifndef ACCESSCONTROL_RECEIVEFSM_H
#define ACCESSCONTROL_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_core_AccessControl/Messages/MessageSet.h"
#include "urn_jaus_jss_core_AccessControl/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"


#include "AccessControl_ReceiveFSM_sm.h"

#include <ros/ros.h>

namespace urn_jaus_jss_core_AccessControl
{

class DllExport AccessControl_ReceiveFSM : public JTS::StateMachine
{
public:
	AccessControl_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM);
	virtual ~AccessControl_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void initAction();
	virtual void resetTimerAction();
	virtual void sendConfirmControlAction(RequestControl msg, std::string arg0, Receive::Body::ReceiveRec transportData);
	virtual void sendRejectControlAction(ReleaseControl msg, std::string arg0, Receive::Body::ReceiveRec transportData);
	virtual void sendRejectControlToControllerAction(std::string arg0);
	virtual void sendReportAuthorityAction(QueryAuthority msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportControlAction(QueryControl msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportTimeoutAction(QueryTimeout msg, Receive::Body::ReceiveRec transportData);
	virtual void setAuthorityAction(RequestControl msg);
	virtual void setAuthorityAction(SetAuthority msg);
	virtual void storeAddressAction(Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	virtual bool isAuthorityValid(SetAuthority msg);
	virtual bool isControlAvailable();
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
	virtual bool isCurrentAuthorityLess(RequestControl msg);
	virtual bool isDefaultAuthorityGreater(RequestControl msg);
	virtual bool isEmergencyClient(Receive::Body::ReceiveRec transportData);


	void store_emergency_address(JausAddress address);
	void delete_emergency_address(JausAddress address);
	bool has_emergency_address(JausAddress address);
	void timeout(void* arg);
	JausAddress current_controller() {
		return p_current_controller;
	}

	AccessControl_ReceiveFSMContext *context;

protected:
	DeVivo::Junior::JrTimer *p_timer;
	JausAddress p_current_controller;
	jUnsignedByte p_current_authority;
	jUnsignedByte p_default_authority;
	int p_default_timeout;
	std::set<unsigned int> p_emergency_address;
	ros::Publisher p_is_controlled_publisher;
    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;

	JTS::InternalEvent *p_timeout_event;

	void pPublishControlState(bool state);

};

};

#endif // ACCESSCONTROL_RECEIVEFSM_H
