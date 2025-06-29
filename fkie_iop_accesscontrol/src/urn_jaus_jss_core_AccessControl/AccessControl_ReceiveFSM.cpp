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


#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_component/iop_component.hpp>

using namespace JTS;

namespace urn_jaus_jss_core_AccessControl
{


AccessControl_ReceiveFSM::AccessControl_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("AccessControl")),
  p_timer(std::chrono::seconds(60), std::bind(&AccessControl_ReceiveFSM::pTimeout, this), true)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	p_current_authority = 0;
	p_default_authority = 1;
	p_default_timeout = 60;
	p_ros_available = true;
	p_is_new_controller = false;
	p_timeout_event = new InternalEvent("Timedout", "ControlTimeout");
	context = new AccessControl_ReceiveFSMContext(*this);

	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
}



AccessControl_ReceiveFSM::~AccessControl_ReceiveFSM()
{
	p_timer.stop();
	delete context;
	delete p_timeout_event;
}

void AccessControl_ReceiveFSM::setupNotifications()
{
	pEvents_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "Events_ReceiveFSM");
	pEvents_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "Events_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving_Ready", "AccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving_Ready", "AccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving_Ready", "AccessControl_ReceiveFSM");
	registerNotification("Receiving", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving", "AccessControl_ReceiveFSM");

}

void AccessControl_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "AccessControl");
	cfg.declare_param<int64_t>("access_timeout", p_default_timeout, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
		"Time period in seconds after which the exclusive control goes lost. Zero disables the timeout.",
		"Default: 60 sec");
	cfg.param<int64_t>("access_timeout", p_default_timeout, p_default_timeout);
	uint8_t da = p_default_authority;
	cfg.declare_param<uint8_t>("default_authority", p_default_authority, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
		"The authority level of a client requesting control (RequestControl) must be greather than or equal to the this value.",
		"Default: 1; Possible values: 1-254");
	cfg.param<uint8_t>("default_authority", da, da);
	p_default_authority = da;
	if (p_default_timeout > 0) {
		p_timer.set_interval(std::chrono::seconds(p_default_timeout));
	}
	p_report_timeout.getBody()->getReportTimoutRec()->setTimeout(p_default_timeout);
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryAuthority::ID);
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryControl::ID);
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryTimeout::ID);
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryTimeout::ID, &p_report_timeout);
	p_is_controlled_publisher = cfg.create_publisher<std_msgs::msg::Bool>("is_controlled", 5);
	p_is_control_available = cfg.create_publisher<std_msgs::msg::Bool>("is_control_available", 5);
	p_sub_control_available = cfg.create_subscription<std_msgs::msg::Bool>("set_control_available", 5, std::bind(&AccessControl_ReceiveFSM::p_set_control_available, this, std::placeholders::_1));
	pPublishControlState(false);
	auto msg = std_msgs::msg::Bool();
	msg.data = p_ros_available;
	p_is_control_available->publish(msg);
}

void AccessControl_ReceiveFSM::pTimeout()
{
	RCLCPP_DEBUG(logger, "control timedout to %s", p_current_controller.str().c_str());
	this->getHandler()->invoke(p_timeout_event);
	// create a new event, since the InternalEventHandler deletes the given.
	p_timeout_event = new InternalEvent("Timedout", "ControlTimeout");
}

void AccessControl_ReceiveFSM::initAction()
{
	/// Insert User Code HERE
	setAuthority(p_default_authority);
}

void AccessControl_ReceiveFSM::resetTimerAction()
{
	/// Insert User Code HERE
	RCLCPP_DEBUG(logger, "ResetTimerAction: restart timer");
	p_timer.stop();
	if (p_default_timeout > 0) {
		p_timer.start();
	}
}

void AccessControl_ReceiveFSM::sendConfirmControlAction(RequestControl msg, std::string arg0, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "sendConfirmControlAction to %s, code: %s", sender.str().c_str(), arg0.c_str());
	ConfirmControl confirm_msg;
	uint8_t responseCode = 0;
	if (arg0 == "CONTROL_ACCEPTED") {
		responseCode = 0;
		// timer reset by internal event "ResetTimerAction"
	} else if (arg0 == "NOT_AVAILABLE") {
		responseCode = 1;
	} else if (arg0 == "INSUFFICIENT_AUTHORITY") {
		responseCode = 2;
	} else {
		RCLCPP_WARN(logger, "sendConfirmControlAction unknown code: %s", arg0.c_str());
	}
	confirm_msg.getBody()->getConfirmControlRec()->setResponseCode(responseCode);
	// Now send it to the requesting component
	sendJausMessage( confirm_msg, sender, Priority::High );
}

void AccessControl_ReceiveFSM::sendRejectControlAction(ReleaseControl msg, std::string arg0, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "sendRejectControlAction to %s, code: %s", sender.str().c_str(), arg0.c_str());
	RejectControl reject_msg;
	if (arg0 == "CONTROL_RELEASED") {
		reject_msg.getBody()->getRejectControlRec()->setResponseCode(0);
		if (p_current_controller.get() != 0) {
			RCLCPP_DEBUG(logger, "delete CONTROLER");
			setControl(JausAddress(0));
			setAuthority(p_default_authority);
			p_timer.stop();
			pPublishControlState(false);
		}
	} else if (arg0 == "NOT_AVAILABLE") {
		RCLCPP_DEBUG(logger, "  access control not available");
		pPublishControlState(false);
		reject_msg.getBody()->getRejectControlRec()->setResponseCode(1);
	} else {
		RCLCPP_WARN(logger, "sendRejectControlAction unknown code: %s", arg0.c_str());
	}
	// Now send it to the requesting component
	sendJausMessage( reject_msg, sender, Priority::High );
}

void AccessControl_ReceiveFSM::sendRejectControlToControllerAction(std::string arg0)
{
	if (p_current_controller.get() != 0) {
		RCLCPP_DEBUG(logger, "sendRejectControlToControllerAction to %s, code: %s", p_current_controller.str().c_str(), arg0.c_str());
		JausAddress dest = p_current_controller;
		RejectControl reject_msg;
		if (arg0 == "CONTROL_RELEASED") {
			RCLCPP_DEBUG(logger, "delete current CONTROLER");
			setControl(JausAddress(0));
			setAuthority(p_default_authority);
			p_timer.stop();
			pPublishControlState(false);
			reject_msg.getBody()->getRejectControlRec()->setResponseCode(0);
		} else if (arg0 == "NOT_AVAILABLE") {
			RCLCPP_DEBUG(logger, "  access control not available in e.g. emergency");
			pPublishControlState(false);
			reject_msg.getBody()->getRejectControlRec()->setResponseCode(1);
		} else {
			RCLCPP_WARN(logger, "sendRejectControlToControllerAction unknown code: %s", arg0.c_str());
		}
		// Now send it to the requesting component
		sendJausMessage(reject_msg, dest);
	} else {
		RCLCPP_DEBUG(logger, "sendRejectControlToControllerAction not controller");
	}

}

void AccessControl_ReceiveFSM::sendReportAuthorityAction(QueryAuthority msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "sendReportAuthorityAction to %s, authority: %d", sender.str().c_str(), p_current_authority);
	ReportAuthority response;
	response.getBody()->getReportAuthorityRec()->setAuthorityCode(p_current_authority);
	sendJausMessage(response, sender);
}

void AccessControl_ReceiveFSM::sendReportControlAction(QueryControl msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ReportControl response;
	unsigned int sid = 0;
	unsigned int nid = 0;
	unsigned int cid = 0;
	if (p_current_controller.get() != 0) {
		sid = p_current_controller.getSubsystemID();
		nid = p_current_controller.getNodeID();
		cid = p_current_controller.getComponentID();
	} else if (p_emergency_address.size()) {
		JausAddress eaddr(*p_emergency_address.begin());
		sid = eaddr.getSubsystemID();
		nid = eaddr.getNodeID();
		cid = eaddr.getComponentID();
	}
	response.getBody()->getReportControlRec()->setSubsystemID(sid);
	response.getBody()->getReportControlRec()->setNodeID(nid);
	response.getBody()->getReportControlRec()->setComponentID(cid);
	response.getBody()->getReportControlRec()->setAuthorityCode(p_current_authority);
	RCLCPP_DEBUG(logger, "Sending ReportControl to %s; current controller: %d.%d.%d", sender.str().c_str(), sid, nid, cid);
	sendJausMessage(response, sender, Priority::High);
}

void AccessControl_ReceiveFSM::sendReportTimeoutAction(QueryTimeout msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	jUnsignedByte timeout = p_default_timeout;
	RCLCPP_DEBUG(logger, "send AccessTimeout to %s, timeout: %d", sender.str().c_str(), timeout);
	ReportTimeout response;
	response.getBody()->getReportTimoutRec()->setTimeout(timeout);
	sendJausMessage(response, sender);
}

void AccessControl_ReceiveFSM::setAuthorityAction(RequestControl msg)
{
	/// Insert User Code HERE
	setAuthority(msg.getBody()->getRequestControlRec()->getAuthorityCode());
	RCLCPP_DEBUG(logger, "setAuthotityAction while RequestControl to %d", p_current_authority);
}

void AccessControl_ReceiveFSM::setAuthorityAction(SetAuthority msg)
{
	/// Insert User Code HERE
	setAuthority(msg.getBody()->getAuthorityRec()->getAuthorityCode());
	RCLCPP_DEBUG(logger, "setAuthorityAction to %d", p_current_authority);
}

void AccessControl_ReceiveFSM::storeAddressAction(Receive::Body::ReceiveRec transportData)
{
	setControl(transportData.getAddress());
	RCLCPP_DEBUG(logger, "store address of controlling component as %s", p_current_controller.str().c_str());
	pPublishControlState(true);
}



bool AccessControl_ReceiveFSM::isAuthorityValid(SetAuthority msg)
{
	/// Insert User Code HERE
	if ((msg.getBody()->getAuthorityRec()->getAuthorityCode() <= p_current_authority)
		&& msg.getBody()->getAuthorityRec()->getAuthorityCode() >= p_default_authority) {
		return true;
	}
	return false;
}
bool AccessControl_ReceiveFSM::isControlAvailable()
{
	return (p_emergency_address.size() == 0 && p_ros_available);
}
bool AccessControl_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	if (p_current_controller == transportData.getAddress()) {
		return true;
	}
	return false;
}
bool AccessControl_ReceiveFSM::isCurrentAuthorityLess(RequestControl msg)
{
	if (p_current_authority < msg.getBody()->getRequestControlRec()->getAuthorityCode())
		return true;
	return false;
}
bool AccessControl_ReceiveFSM::isDefaultAuthorityGreater(RequestControl msg)
{
	if (p_default_authority > msg.getBody()->getRequestControlRec()->getAuthorityCode())
		return true;
	return false;
}

bool AccessControl_ReceiveFSM::isEmergencyClient(Receive::Body::ReceiveRec transportData)
{
	return has_emergency_address(transportData.getAddress());
}

void AccessControl_ReceiveFSM::setAuthority(uint8_t authority)
{
	if (authority != p_current_authority) {
		p_current_authority = authority;
		p_report_authority.getBody()->getReportAuthorityRec()->setAuthorityCode(p_current_authority);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryAuthority::ID, &p_report_authority);
	}
	if (p_is_new_controller) {
		p_is_new_controller = false;
		p_report_control.getBody()->getReportControlRec()->setAuthorityCode(p_current_authority);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryControl::ID, &p_report_control);
	}
}

void AccessControl_ReceiveFSM::setControl(JausAddress address)
{
	if (p_current_controller != address) {
		p_current_controller = address;
		p_report_control.getBody()->getReportControlRec()->setSubsystemID(p_current_controller.getSubsystemID());
		p_report_control.getBody()->getReportControlRec()->setNodeID(p_current_controller.getNodeID());
		p_report_control.getBody()->getReportControlRec()->setComponentID(p_current_controller.getComponentID());
		p_is_new_controller = true;
	}
}

void AccessControl_ReceiveFSM::store_emergency_address(JausAddress address)
{
	p_emergency_address.insert(address.get());
}

void AccessControl_ReceiveFSM::delete_emergency_address(JausAddress address)
{
	std::set<unsigned int>::iterator res = p_emergency_address.find(address.get());
	if (res != p_emergency_address.end()) {
		p_emergency_address.erase(address.get());
	}
}

bool AccessControl_ReceiveFSM::has_emergency_address(JausAddress address)
{
	std::set<unsigned int>::iterator res = p_emergency_address.find(address.get());
	return (res != p_emergency_address.end());
}

void AccessControl_ReceiveFSM::pPublishControlState(bool state)
{
	auto ros_msg = std_msgs::msg::Bool();
	ros_msg.data = state;
	p_is_controlled_publisher->publish(ros_msg);
}

void AccessControl_ReceiveFSM::p_set_control_available(const std_msgs::msg::Bool::SharedPtr msg)
{
	p_ros_available = msg->data;
	auto pubmsg = std_msgs::msg::Bool();
	pubmsg.data = p_ros_available;
	p_is_control_available->publish(pubmsg);
}
}
