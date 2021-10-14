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


#ifndef ACCESSCONTROLCLIENT_RECEIVEFSM_H
#define ACCESSCONTROLCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_core_AccessControlClient/Messages/MessageSet.h"
#include "urn_jaus_jss_core_AccessControlClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"


#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <ros/ros.h>
#include <fkie_iop_msgs/JausAddress.h>
#include <std_msgs/UInt8.h>
#include <fkie_iop_accesscontrol/RemoteComponent.h>
#include <fkie_iop_accesscontrol/RemoteComponentList.h>
#include "AccessControlClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_core_AccessControlClient
{

class DllExport AccessControlClient_ReceiveFSM : public JTS::StateMachine
{
public:
	static unsigned char ACCESS_STATE_NOT_AVAILABLE;
	static unsigned char ACCESS_STATE_NOT_CONTROLLED;
	static unsigned char ACCESS_STATE_CONTROL_RELEASED;
	static unsigned char ACCESS_STATE_CONTROL_ACCEPTED;
	static unsigned char ACCESS_STATE_TIMEOUT;
	static unsigned char ACCESS_STATE_INSUFFICIENT_AUTHORITY;

	AccessControlClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM);
	virtual ~AccessControlClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleConfirmControlAction(ConfirmControl msg, Receive::Body::ReceiveRec transportData);
	virtual void handleRejectControlAction(RejectControl msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportAuthorityAction(ReportAuthority msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportControlAction(ReportControl msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportTimeoutAction(ReportTimeout msg, Receive::Body::ReceiveRec transportData);
	virtual void resetControlTimerAction();


	/// Guard Methods

	template<class T>
	void requestAccess(JausAddress address, void(T::*reply_handler)(JausAddress &, unsigned char code), T*obj, jUnsignedByte authority=255)
	{
		boost::function<void (JausAddress &, unsigned char code)> callback = boost::bind(reply_handler, obj, _1, _2);;
		p_reply_callbacks[address.get()].push_back(callback);
		pRequestAccess(address, authority);
	}
	void requestAccess(JausAddress address, jUnsignedByte authority=255)
	{
		pRequestAccess(address, authority);
	}
	template<class T>
	void releaseAccess(JausAddress address, void(T::*reply_handler)(JausAddress &, unsigned char code), T*obj)
	{
		boost::function<void (JausAddress &, unsigned char code)> callback = boost::bind(reply_handler, obj, _1, _2);;
		p_reply_callbacks[address.get()].push_back(callback);
		pReleaseAccess(address);
	}
	void releaseAccess(JausAddress address)
	{
		pReleaseAccess(address);
	}
	bool hasAccess(JausAddress address);
	template<class T>
	void add_reply_handler(void(T::*handler)(JausAddress &, unsigned char code), T*obj) {
		p_reply_handler.push_back(boost::bind(handler, obj, _1, _2));
	}
	void set_emergency_subsystem(unsigned short subsystem_id) { p_emergency_subsystem = subsystem_id; }

	AccessControlClient_ReceiveFSMContext *context;

protected:
	/// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;

	std::vector<boost::function<void (JausAddress &, unsigned char code)> > p_reply_handler;
	std::map <unsigned int, std::vector<boost::function<void (JausAddress &, unsigned char code)> > > p_reply_callbacks;  // unsigned int -> JausAddress::get(), list with callbacks to this address
	boost::function<void (JausAddress &, unsigned char code)> p_class_access_reply_callback;
	unsigned short p_emergency_subsystem;
	jUnsignedByte p_default_timeout;
	iop::RemoteComponentList p_remote_components;
	ros::WallTimer p_timer;
	JTS::InternalEvent *p_timeout_event;
	QueryControl p_query_control;
	ros::Publisher p_pub_current_controller;
	ros::Publisher p_pub_current_authority;
	void pTimeoutCallback(const ros::WallTimerEvent& event);
	void pInformReplyCallbacks(JausAddress &address, unsigned char code);
	void pRequestAccess(JausAddress address, jUnsignedByte authority=255);
	void pReleaseAccess(JausAddress address);
};

};

#endif // ACCESSCONTROLCLIENT_RECEIVEFSM_H
