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


#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include <iop_component_fkie/iop_config.h>

#include <ros/console.h>

using namespace JTS;

namespace urn_jaus_jss_core_AccessControlClient
{

unsigned char AccessControlClient_ReceiveFSM::ACCESS_STATE_NOT_AVAILABLE          = 0;
unsigned char AccessControlClient_ReceiveFSM::ACCESS_STATE_NOT_CONTROLLED         = 1;
unsigned char AccessControlClient_ReceiveFSM::ACCESS_STATE_CONTROL_RELEASED       = 2;
unsigned char AccessControlClient_ReceiveFSM::ACCESS_STATE_CONTROL_ACCEPTED       = 3;
unsigned char AccessControlClient_ReceiveFSM::ACCESS_STATE_TIMEOUT                = 4;
unsigned char AccessControlClient_ReceiveFSM::ACCESS_STATE_INSUFFICIENT_AUTHORITY = 5;


AccessControlClient_ReceiveFSM::AccessControlClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new AccessControlClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	p_default_timeout = 1;
	p_timeout_event = new InternalEvent("Timeout", "ControlTimeout");
	ros::NodeHandle nh;
	p_timer = nh.createWallTimer(ros::WallDuration(p_default_timeout), &AccessControlClient_ReceiveFSM::pTimeoutCallback, this, false, false);
}



AccessControlClient_ReceiveFSM::~AccessControlClient_ReceiveFSM()
{
	p_timer.stop();
	delete context;
	delete p_timeout_event;
}

void AccessControlClient_ReceiveFSM::setupNotifications()
{
	pEventsClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	pEventsClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	registerNotification("Receiving", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving", "AccessControlClient_ReceiveFSM");
	iop::Config cfg("~AccessControlClient");
	p_pub_current_controller = cfg.advertise_p<iop_msgs_fkie::JausAddress>("current_controller", 5, true);
	p_pub_current_authority = cfg.advertise_p<std_msgs::UInt8>("current_authority", 5, true);
	p_timer.start();
}

void AccessControlClient_ReceiveFSM::handleConfirmControlAction(ConfirmControl msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	jUnsignedByte rcode = msg.getBody()->getConfirmControlRec()->getResponseCode();
	if (rcode == 0) {
		ROS_DEBUG_NAMED("AccessControlClient", "CONTROL_ACCEPTED: %s", sender.str().c_str());
		if (!hasAccess(sender)) {
			QueryTimeout rt_msg;
			this->sendJausMessage(rt_msg, sender);
		}
		p_remote_components.set_ack(sender, ros::WallTime::now().toSec());
		pInformReplyCallbacks(sender, ACCESS_STATE_CONTROL_ACCEPTED);
	} else if (rcode == 1) {
		ROS_WARN_NAMED("AccessControlClient", "NOT_AVAILABLE: %s", sender.str().c_str());
		pInformReplyCallbacks(sender, ACCESS_STATE_NOT_AVAILABLE);
	} else if (rcode == 2) {
		ROS_WARN_NAMED("AccessControlClient", "INSUFFICIENT_AUTHORITY: %s", sender.str().c_str());
		QueryAuthority qa_msg;
		this->sendJausMessage(qa_msg, sender);
		pInformReplyCallbacks(sender, ACCESS_STATE_INSUFFICIENT_AUTHORITY);
	}
	this->sendJausMessage(p_query_control, sender);
}

void AccessControlClient_ReceiveFSM::handleRejectControlAction(RejectControl msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("AccessControlClient", "Control Rejected %s, code: %d", sender.str().c_str(), (int)msg.getBody()->getRejectControlRec()->getResponseCode());
	p_remote_components.remove(sender);
	if (msg.getBody()->getRejectControlRec()->getResponseCode() == 0) {
		pInformReplyCallbacks(sender, ACCESS_STATE_CONTROL_RELEASED);
	} else if (msg.getBody()->getRejectControlRec()->getResponseCode() == 1) {
		pInformReplyCallbacks(sender, ACCESS_STATE_NOT_AVAILABLE);
	}
}

void AccessControlClient_ReceiveFSM::handleReportAuthorityAction(ReportAuthority msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("AccessControlClient", "Current Authority of %s: %d",
			sender.str().c_str(), msg.getBody()->getReportAuthorityRec()->getAuthorityCode());
}

void AccessControlClient_ReceiveFSM::handleReportControlAction(ReportControl msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	uint16_t csubsystem_id = msg.getBody()->getReportControlRec()->getSubsystemID();
	uint8_t cnode_id = msg.getBody()->getReportControlRec()->getNodeID();
	uint8_t ccomponent_id = msg.getBody()->getReportControlRec()->getComponentID();
	uint8_t cauth = msg.getBody()->getReportControlRec()->getAuthorityCode();
	ROS_DEBUG_NAMED("AccessControlClient", "Current controller  %d.%d.%d(%d) of %s",
			csubsystem_id, cnode_id, ccomponent_id, cauth, sender.str().c_str());
	iop_msgs_fkie::JausAddress ros_msg;
	ros_msg.subsystem_id = csubsystem_id;
	ros_msg.node_id = cnode_id;
	ros_msg.component_id = ccomponent_id;
	std_msgs::UInt8 ros_msg_auth;
	ros_msg_auth.data = cauth;
	p_pub_current_controller.publish(ros_msg);
	p_pub_current_authority.publish(ros_msg_auth);
}

void AccessControlClient_ReceiveFSM::handleReportTimeoutAction(ReportTimeout msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	int rtimeout = msg.getBody()->getReportTimoutRec()->getTimeout();
	ROS_INFO_NAMED("AccessControlClient", "received access timeout for %s: %d sec", sender.str().c_str(), rtimeout);
	p_remote_components.set_timeout(sender, rtimeout / 2.0);
}

void AccessControlClient_ReceiveFSM::resetControlTimerAction()
{
	try {
		// determine timouted components
		std::vector<boost::shared_ptr<iop::RemoteComponent> > tdcmpts = p_remote_components.timeouted();
		for (unsigned int cidx = 0; cidx < tdcmpts.size(); cidx++) {
			boost::shared_ptr<iop::RemoteComponent> cmp = tdcmpts[cidx];
			// send request, request time was set while time_to_send_ack()
			JausAddress address = cmp->address();
			ROS_DEBUG_NAMED("AccessControlClient", "Timeouted requests to %s, authority: %d", address.str().c_str(), cmp->authority());
			pInformReplyCallbacks(address, ACCESS_STATE_TIMEOUT);
		}
		// send requests
		std::vector<boost::shared_ptr<iop::RemoteComponent> > cmpts = p_remote_components.time_to_send_request((float)p_default_timeout + 1.0);
		for (unsigned int cidx = 0; cidx < cmpts.size(); cidx++) {
			boost::shared_ptr<iop::RemoteComponent> cmp = cmpts[cidx];
			// send request, request time was set while time_to_send_ack()
			JausAddress address = cmp->address();
			ROS_DEBUG_NAMED("AccessControlClient", "Send request access to %s, authority: %d", address.str().c_str(), cmp->authority());
			RequestControl msg;
			msg.getBody()->getRequestControlRec()->setAuthorityCode(cmp->authority());
			this->sendJausMessage(msg, address);
		}
	} catch (std::exception &e) {
		ROS_WARN_NAMED("AccessControlClient", "timer error: %s", e.what());
	}
}

void AccessControlClient_ReceiveFSM::pRequestAccess(JausAddress address, jUnsignedByte authority)
{
	p_remote_components.create(address, authority);
	ROS_DEBUG_NAMED("AccessControlClient", "Send request access to %s, authority: %d", address.str().c_str(), authority);
	RequestControl msg;
	msg.getBody()->getRequestControlRec()->setAuthorityCode(authority);
	this->sendJausMessage(p_query_control, address);
	this->sendJausMessage(msg, address);
}

void AccessControlClient_ReceiveFSM::pReleaseAccess(JausAddress address)
{
	ROS_DEBUG_NAMED("AccessControlClient", "Send release access to %s", address.str().c_str());
	p_remote_components.remove(address);
	ReleaseControl msg;
	this->sendJausMessage(msg, address);
}

bool AccessControlClient_ReceiveFSM::hasAccess(JausAddress address)
{
	return p_remote_components.has_access(address);
}

void AccessControlClient_ReceiveFSM::pTimeoutCallback(const ros::WallTimerEvent& event)
{
	this->getHandler()->invoke(p_timeout_event);
	// create a new event, since the InternalEventHandler deletes the given.
	p_timeout_event = new InternalEvent("Timeout", "ControlTimeout");
}


void AccessControlClient_ReceiveFSM::pInformReplyCallbacks(JausAddress &address, unsigned char code)
{
	std::map <unsigned int, std::vector<boost::function<void (JausAddress &, unsigned char code)> > >::iterator it;
	it = p_reply_callbacks.find(address.get());
	if (it != p_reply_callbacks.end()) {
		for (unsigned int i = 0; i < it->second.size(); i++) {
			it->second[i](address, code);
		}
		it->second.clear();
		p_reply_callbacks.erase(it);
	}
	if (!p_class_access_reply_callback.empty()) {
			p_class_access_reply_callback(address, code);
	}
	std::vector<boost::function<void (JausAddress &, unsigned char code)> >::iterator it_all;
	for (it_all = p_reply_handler.begin(); it_all != p_reply_handler.end(); ++it_all) {
		(*it_all)(address, code);
	}
}


};
