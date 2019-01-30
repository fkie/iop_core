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


#ifndef MANAGEMENT_RECEIVEFSM_H
#define MANAGEMENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_core_Management/Messages/MessageSet.h"
#include "urn_jaus_jss_core_Management/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"


#include "Management_ReceiveFSM_sm.h"
#include <ros/ros.h>

namespace urn_jaus_jss_core_Management
{
/**
 * Internal status, see StatusReport
 **/
const int STATE_INITIALIZE = 0;
const int STATE_READY = 1;
const int STATE_STANDBY = 2;
const int STATE_SHUTDOWN = 3;
const int STATE_FAILURE = 4;
const int STATE_EMERGENCY = 5;

class DllExport Management_ReceiveFSM : public JTS::StateMachine
{
public:
	Management_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM);
	virtual ~Management_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void deleteIDAction(Receive::Body::ReceiveRec transportData);
	virtual void emergencyAction();
	virtual void failureAction();
	virtual void goReadyAction();
	virtual void goStandbyAction();
	virtual void initializeAction();
	virtual void readyAction();
	virtual void resetAction();
	virtual void resetEmergencyAction();
	virtual void sendRejectControlToControllerAction(std::string arg0);
	virtual void sendReportStatusAction(QueryStatus msg, Receive::Body::ReceiveRec transportData);
	virtual void shutdownAction();
	virtual void standbyAction();
	virtual void storeIDAction(Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
	virtual bool isEmergencyCleared();
	virtual bool isIDStored(Receive::Body::ReceiveRec transportData);



	Management_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;

	ReportStatus p_report_status;
	ros::Publisher p_pub_emergency;
	ros::Publisher p_pub_ready;
	jUnsignedByte p_state;
	void pSetState(jUnsignedByte state);
};

};

#endif // MANAGEMENT_RECEIVEFSM_H
