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


#include "urn_jaus_jss_core_Management/Management_ReceiveFSM.h"
#include <fkie_iop_component/iop_component.hpp>
#include <fkie_iop_component/iop_config.hpp>

using namespace JTS;

namespace urn_jaus_jss_core_Management
{



Management_ReceiveFSM::Management_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("Management"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new Management_ReceiveFSMContext(*this);
//	context->setDebugFlag(true);

	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_state = 0;
}



Management_ReceiveFSM::~Management_ReceiveFSM()
{
	delete context;

}

void Management_ReceiveFSM::setupNotifications()
{
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Init", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Standby", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Init", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Init", "AccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Init", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Standby", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Failure", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Shutdown", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Emergency", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Standby", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Ready", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Failure", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready", "Management_ReceiveFSM");
	registerNotification("Receiving", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving", "Management_ReceiveFSM");

}

void Management_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "Management");
	p_pub_emergency = cfg.create_publisher<std_msgs::msg::Bool>("is_emergency", 5);
	p_pub_ready = cfg.create_publisher<std_msgs::msg::Bool>("is_ready", 5);
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryStatus::ID);
	auto rosmsg = std_msgs::msg::Bool();
	rosmsg.data = false;
	p_pub_emergency->publish(rosmsg);
	p_pub_ready->publish(rosmsg);
}

void Management_ReceiveFSM::deleteIDAction(Receive::Body::ReceiveRec transportData)
{
	JausAddress emergency_client = transportData.getAddress();
	RCLCPP_DEBUG(logger, "reset emergency -> delete ID %s", emergency_client.str().c_str());
	pAccessControl_ReceiveFSM->delete_emergency_address(emergency_client);
	if (pAccessControl_ReceiveFSM->isControlAvailable() && p_state == STATE_EMERGENCY) {
		RCLCPP_DEBUG(logger, "reset emergency -> change into initialized state");
		ieHandler->invoke(new Released());
	} else if (p_state == STATE_EMERGENCY) {
		RCLCPP_DEBUG(logger, "reset emergency -> there are further clients which holds emergency");
	}
}

void Management_ReceiveFSM::emergencyAction()
{
	RCLCPP_DEBUG(logger, "emergencyAction");
	pSetState(STATE_EMERGENCY);
}

void Management_ReceiveFSM::failureAction()
{
	RCLCPP_DEBUG(logger, "failureAction");
	pSetState(STATE_FAILURE);
}

void Management_ReceiveFSM::goReadyAction()
{
	/// Insert User Code HERE
	RCLCPP_DEBUG(logger, "goReadyAction");
}

void Management_ReceiveFSM::goStandbyAction()
{
	/// Insert User Code HERE
	RCLCPP_DEBUG(logger, "goStandbyAction");
}


void Management_ReceiveFSM::initializeAction()
{
	/// Insert User Code HERE
	RCLCPP_DEBUG(logger, "initializeAction");
	pSetState(STATE_INITIALIZE);
	ieHandler->invoke(new Initialized());
}

void Management_ReceiveFSM::readyAction()
{
	RCLCPP_DEBUG(logger, "readyAction");
	pSetState(STATE_READY);
}

void Management_ReceiveFSM::resetAction()
{
	RCLCPP_DEBUG(logger, "resetAction");
	pSetState(STATE_INITIALIZE);
}

void Management_ReceiveFSM::resetEmergencyAction()
{
	RCLCPP_DEBUG(logger, "resetEmergencyAction");
	if (!pAccessControl_ReceiveFSM->isControlAvailable()) {
		RCLCPP_DEBUG(logger, "can reset emergency if not all client accepted!");
		throw std::logic_error("can reset emergency if not all client accepted");
	}
	pSetState(STATE_INITIALIZE);
}

void Management_ReceiveFSM::sendRejectControlToControllerAction(std::string arg0)
{
	RCLCPP_DEBUG(logger, "sendRejectControlToControllerAction: %s", arg0.c_str());
	pAccessControl_ReceiveFSM->sendRejectControlToControllerAction(arg0);
}

void Management_ReceiveFSM::sendReportStatusAction(QueryStatus msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "Send ReportStatus (%d) to %s", (int)p_state, sender.str().c_str());
	sendJausMessage(p_report_status, sender);
}

void Management_ReceiveFSM::shutdownAction()
{
	RCLCPP_DEBUG(logger, "SHUTDOWN");
	pSetState(STATE_SHUTDOWN);
}

void Management_ReceiveFSM::standbyAction()
{
	RCLCPP_DEBUG(logger, "standbyAction");
	pSetState(STATE_STANDBY);
}

void Management_ReceiveFSM::storeIDAction(Receive::Body::ReceiveRec transportData)
{
	JausAddress emergency_client = transportData.getAddress();
	if (!pAccessControl_ReceiveFSM->has_emergency_address(emergency_client)) {
		RCLCPP_DEBUG(logger, "emergency -> store ID %s", emergency_client.str().c_str());
		pAccessControl_ReceiveFSM->store_emergency_address(emergency_client);
	}

}

bool Management_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

bool Management_ReceiveFSM::isEmergencyCleared()
{
	return pAccessControl_ReceiveFSM->isControlAvailable();
}

bool Management_ReceiveFSM::isIDStored(Receive::Body::ReceiveRec transportData)
{
	return pAccessControl_ReceiveFSM->has_emergency_address(transportData.getAddress());
}

void Management_ReceiveFSM::pSetState(jUnsignedByte state)
{
	if (state != p_state) {
		if (p_state == STATE_EMERGENCY) {
			auto rosmsg = std_msgs::msg::Bool();
			rosmsg.data = false;
			p_pub_emergency->publish(rosmsg);
		} else if (state == STATE_EMERGENCY) {
			auto rosmsg = std_msgs::msg::Bool();
			rosmsg.data = true;
			p_pub_emergency->publish(rosmsg);
		} else if (p_state == STATE_READY) {
			auto rosmsg = std_msgs::msg::Bool();
			rosmsg.data = false;
			p_pub_ready->publish(rosmsg);
		} else if (state == STATE_READY) {
			auto rosmsg = std_msgs::msg::Bool();
			rosmsg.data = true;
			p_pub_ready->publish(rosmsg);
		}
		p_state = state;
		p_report_status.getBody()->getReportStatusRec()->setStatus(p_state);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryStatus::ID, &p_report_status);
	}
}

}
