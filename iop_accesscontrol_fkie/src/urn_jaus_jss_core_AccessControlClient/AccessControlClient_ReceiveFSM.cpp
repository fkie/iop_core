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
	p_timer.start();
}

void AccessControlClient_ReceiveFSM::handleConfirmControlAction(ConfirmControl msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSourceID()->getSubsystemID();
	uint8_t node_id = transportData.getSourceID()->getNodeID();
	uint8_t component_id = transportData.getSourceID()->getComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	jUnsignedByte rcode = msg.getBody()->getConfirmControlRec()->getResponseCode();
	if (rcode == 0) {
		ROS_DEBUG_NAMED("AccessControlClient", "CONTROL_ACCEPTED: %d.%d.%d", subsystem_id, node_id, component_id);
		if (!hasAccess(sender)) {
			mutex.lock();
			p_controlled_clients[sender.get()] = ros::WallTime::now().toSec();
			if (p_timeouts.find(sender.get()) == p_timeouts.end()) {
					p_timeouts[sender.get()] = 10;
			}
			mutex.unlock();
			QueryTimeout rt_msg;
			this->sendJausMessage(rt_msg, sender);
			p_timer.start();
		}
		pInformReplyCallbacks(sender, ACCESS_STATE_CONTROL_ACCEPTED);
	} else if (rcode == 1) {
		ROS_WARN_NAMED("AccessControlClient", "NOT_AVAILABLE: %d.%d.%d", subsystem_id, node_id, component_id);
		pInformReplyCallbacks(sender, ACCESS_STATE_NOT_AVAILABLE);
	} else if (rcode == 2) {
		ROS_WARN_NAMED("AccessControlClient", "INSUFFICIENT_AUTHORITY: %d.%d.%d", subsystem_id, node_id, component_id);
		QueryAuthority qa_msg;
		this->sendJausMessage(qa_msg, sender);
		QueryControl qc_msg;
		this->sendJausMessage(qc_msg, sender);
		pInformReplyCallbacks(sender, ACCESS_STATE_INSUFFICIENT_AUTHORITY);
	}
	mutex.lock();
	for (unsigned int i=0; i < p_on_request_clients.size(); i++) {
		if (p_on_request_clients[i].get() == sender.get()) {
			p_on_request_clients.erase(p_on_request_clients.begin()+i);
			break;
		}
	}
	mutex.unlock();
}

void AccessControlClient_ReceiveFSM::handleRejectControlAction(RejectControl msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSourceID()->getSubsystemID();
	uint8_t node_id = transportData.getSourceID()->getNodeID();
	uint8_t component_id = transportData.getSourceID()->getComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("AccessControlClient", "Control Rejected %d.%d.%d, code: %d", subsystem_id, node_id, component_id, (int)msg.getBody()->getRejectControlRec()->getResponseCode());
	pRemoveClient(sender.get());
	if (msg.getBody()->getRejectControlRec()->getResponseCode() == 0) {
		pInformReplyCallbacks(sender, ACCESS_STATE_CONTROL_RELEASED);
	} else if (msg.getBody()->getRejectControlRec()->getResponseCode() == 1) {
		pInformReplyCallbacks(sender, ACCESS_STATE_NOT_AVAILABLE);
	}
}

void AccessControlClient_ReceiveFSM::handleReportAuthorityAction(ReportAuthority msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSourceID()->getSubsystemID();
	uint8_t node_id = transportData.getSourceID()->getNodeID();
	uint8_t component_id = transportData.getSourceID()->getComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("AccessControlClient", "Current Authority of %d.%d.%d: %d",
			subsystem_id, node_id, component_id, msg.getBody()->getReportAuthorityRec()->getAuthorityCode());
}

void AccessControlClient_ReceiveFSM::handleReportControlAction(ReportControl msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSourceID()->getSubsystemID();
	uint8_t node_id = transportData.getSourceID()->getNodeID();
	uint8_t component_id = transportData.getSourceID()->getComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	uint16_t csubsystem_id = msg.getBody()->getReportControlRec()->getSubsystemID();
	uint8_t cnode_id = msg.getBody()->getReportControlRec()->getNodeID();
	uint8_t ccomponent_id = msg.getBody()->getReportControlRec()->getComponentID();
	uint8_t cauth = msg.getBody()->getReportControlRec()->getAuthorityCode();
	ROS_DEBUG_NAMED("AccessControlClient", "Current controller  %d.%d.%d(%d) of %d.%d.%d",
			csubsystem_id, cnode_id, ccomponent_id, cauth, subsystem_id, node_id, component_id);
}

void AccessControlClient_ReceiveFSM::handleReportTimeoutAction(ReportTimeout msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSourceID()->getSubsystemID();
	uint8_t node_id = transportData.getSourceID()->getNodeID();
	uint8_t component_id = transportData.getSourceID()->getComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	int rtimeout = msg.getBody()->getReportTimoutRec()->getTimeout();
	ROS_DEBUG_NAMED("AccessControlClient", "received access timeout for %d.%d.%d: %d sec", subsystem_id, node_id, component_id, rtimeout);
	p_timeouts[sender.get()] = rtimeout / 2.0;
}

void AccessControlClient_ReceiveFSM::resetControlTimerAction()
{
	// TODO: add reaction for timeout -> autorequest until user reject access?
	try {
		// try to avoid an often access request
		mutex.lock();
		std::map <unsigned int, int>::iterator tit;
		for (tit = p_timeouts.begin(); tit != p_timeouts.end(); ++tit) {
			double last_confirm = p_controlled_clients[tit->first];
			double now = ros::WallTime::now().toSec();
			if (last_confirm + (float)tit->second < now + (float)p_default_timeout + 1.0
					and p_addresses.find(tit->first) != p_addresses.end()) {
				// send request
				JausAddress address = p_addresses[tit->first];
				p_controlled_clients[tit->first] = now;
				ROS_DEBUG_NAMED("AccessControlClient", "Send request access to %d.%d.%d, authority: %d", address.getSubsystemID(),
						address.getNodeID(), address.getComponentID(), p_auths[tit->first]);
				RequestControl msg;
				msg.getBody()->getRequestControlRec()->setAuthorityCode(p_auths[tit->first]);
				this->sendJausMessage(msg, address);
			}
		}
		mutex.unlock();
	} catch (std::exception &e) {
		mutex.unlock();
		ROS_WARN_NAMED("AccessControlClient", "timer error: %s", e.what());
	}
}

void AccessControlClient_ReceiveFSM::pRequestAccess(JausAddress address, jUnsignedByte authority)
{
	jUnsignedByte auth = authority;
	bool has_access = hasAccess(address);
	if (!pIsOnAccess(address) && !has_access) {
		mutex.lock();
		p_on_request_clients.push_back(address);
		p_addresses[address.get()] = address;
		p_auths[address.get()] = authority;
		mutex.unlock();
	}
	bool send_request = true;
	//  try {
	//    // try to avoid an often access request
	//    if (hasAccess(address)) {
	//      mutex.lock();
	//      double last_request = p_controlled_clients[address.get()];
	//      double now = ros::WallTime::now().toSec();
	//      if (now - last_request > p_timeouts[address.get()] / 3.0) {
	//        p_controlled_clients[address.get()] = now;
	//      } else {
	//        send_request = false;
	//      }
	//      mutex.unlock();
	//    }
	//  } catch (std::exception &e) {
	//    printf("ERROR %s\n", e.what());
	//    mutex.unlock();
	//  }
	if (send_request) {
		ROS_DEBUG_NAMED("AccessControlClient", "Send request access to %d.%d.%d, authority: %d", address.getSubsystemID(),
				address.getNodeID(), address.getComponentID(), authority);
		RequestControl msg;
		msg.getBody()->getRequestControlRec()->setAuthorityCode(auth);
		this->sendJausMessage(msg, address);
	}
}

void AccessControlClient_ReceiveFSM::pReleaseAccess(JausAddress address)
{
	ROS_DEBUG_NAMED("AccessControlClient", "Send release access to %d.%d.%d", address.getSubsystemID(),
			address.getNodeID(), address.getComponentID());
	pRemoveClient(address.get());
	ReleaseControl msg;
	this->sendJausMessage(msg, address);
}

bool AccessControlClient_ReceiveFSM::hasAccess(JausAddress address)
{
	mutex.lock();
	if (p_controlled_clients.find(address.get()) != p_controlled_clients.end()) {
		mutex.unlock();
		return true;
	}
	mutex.unlock();
	return false;
}

bool AccessControlClient_ReceiveFSM::pIsOnAccess(JausAddress address)
{
	mutex.lock();
	for (unsigned int i=0; i < p_on_request_clients.size(); i++) {
		if (p_on_request_clients[i].get() == address.get()) {
			mutex.unlock();
			return true;
		}
	}
	mutex.unlock();
	return false;
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

void AccessControlClient_ReceiveFSM::pRemoveClient(unsigned int address)
{
	mutex.lock();
	std::map <unsigned int, double>::iterator it = p_controlled_clients.find(address);
	if (it != p_controlled_clients.end()) {
		p_controlled_clients.erase(it);
	}
	std::map <unsigned int, int>::iterator itt = p_timeouts.find(address);
	if (itt != p_timeouts.end()) {
		p_timeouts.erase(itt);
	}
	if (p_controlled_clients.size() == 0) {
		p_timer.stop();
	}
	for (unsigned int i=0; i < p_on_request_clients.size(); i++) {
		if (p_on_request_clients[i].get() == address) {
			p_on_request_clients.erase(p_on_request_clients.begin()+i);
			break;
		}
	}
	mutex.unlock();
}



};
