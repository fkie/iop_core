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


#include "urn_jaus_jss_core_ManagementClient/ManagementClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_component.hpp>
#include <fkie_iop_component/iop_config.hpp>

using namespace JTS;

namespace urn_jaus_jss_core_ManagementClient
{

unsigned char ManagementClient_ReceiveFSM::MANAGEMENT_STATE_INIT      = 0;
unsigned char ManagementClient_ReceiveFSM::MANAGEMENT_STATE_READY     = 1;
unsigned char ManagementClient_ReceiveFSM::MANAGEMENT_STATE_STANDBY   = 2;
unsigned char ManagementClient_ReceiveFSM::MANAGEMENT_STATE_SHUTDOWN  = 3;
unsigned char ManagementClient_ReceiveFSM::MANAGEMENT_STATE_FAILURE   = 4;
unsigned char ManagementClient_ReceiveFSM::MANAGEMENT_STATE_EMERGENCY = 5;
unsigned char ManagementClient_ReceiveFSM::MANAGEMENT_STATE_UNKNOWN   = 255;

ManagementClient_ReceiveFSM::ManagementClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("ManagementClient")),
  p_query_timer(std::chrono::seconds(10), std::bind(&ManagementClient_ReceiveFSM::pQueryCallback, this), false)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new ManagementClient_ReceiveFSMContext(*this);

	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_status = 255;
	p_hz = 0.0;
	by_query = false;
}



ManagementClient_ReceiveFSM::~ManagementClient_ReceiveFSM()
{
	delete context;
}

void ManagementClient_ReceiveFSM::setupNotifications()
{
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving_Ready", "AccessControlClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	registerNotification("Receiving", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving", "ManagementClient_ReceiveFSM");

}

void ManagementClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "ManagementClient");
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Sets how often the reports are requested. If by_query is true hz must be greather then 0. In this case each time a Query message is sent to get a report. If by_query is false an event is created to get Reports. In this case 0 disables the rate and an event of type on_change will be created.",
		"Default: 0 (each new one)");
	cfg.declare_param<bool>("by_query", by_query, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
		"By default the current state will be requested by creating an event. By setting this variable to true the state is requested by query.",
		"Default: false");
	cfg.param("hz", p_hz, p_hz, false);
	cfg.param("by_query", by_query, by_query, true);
	p_pub_status = cfg.create_publisher<std_msgs::msg::String>("mgmt_status", 5);
	p_pub_status_emergency = cfg.create_publisher<std_msgs::msg::Bool>("mgmt_emergency", 5);
	p_sub_cmd_emergency = cfg.create_subscription<std_msgs::msg::Bool>("cmd_mgmt_emergency", 5, std::bind(&ManagementClient_ReceiveFSM::pRosEmergency, this, std::placeholders::_1));
	p_sub_cmd_ready = cfg.create_subscription<std_msgs::msg::Bool>("cmd_mgmt_reset", 5, std::bind(&ManagementClient_ReceiveFSM::pRosReady, this, std::placeholders::_1));
}

void ManagementClient_ReceiveFSM::reportStatusAction(ReportStatus msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	jUnsignedByte new_state = msg.getBody()->getReportStatusRec()->getStatus();
	if (p_status != new_state) {
		p_status = new_state;
		RCLCPP_DEBUG(logger, "new management status: %d (%s)", p_status, p_status_to_str(p_status).c_str());
		auto ros_msg = std_msgs::msg::String();
		ros_msg.data = p_status_to_str(p_status);
		p_pub_status->publish(ros_msg);
		auto ros_msg_emergency = std_msgs::msg::Bool();
		ros_msg_emergency.data = p_status == 5;  // emergency state
		p_pub_status_emergency->publish(ros_msg_emergency);
		//    p_on_status_query = false;
		//    if (p_do_resume and p_status != 3) {
		//      RCLCPP_DEBUG(logger, "  Resume!\n");
		//      p_do_resume = false;
		//      queryStatus(sender);
		//    }
		if (p_class_interface_callback != nullptr && p_current_client == sender) {
			RCLCPP_DEBUG(logger, "  forward management state to handler");
			p_class_interface_callback(sender, p_status);
		}
		if (p_status == 2 && p_current_client == sender) {
			if (pAccessControlClient_ReceiveFSM->hasAccess(p_current_client )) {
				resume(p_current_client);
			}
		}
		if (p_status != MANAGEMENT_STATE_EMERGENCY) {
			p_current_emergency_address = JausAddress();
			pAccessControlClient_ReceiveFSM->set_emergency_client(p_current_emergency_address);

		}
	}
}

void ManagementClient_ReceiveFSM::queryStatus(JausAddress address)
{
	QueryStatus msg;
	sendJausMessage( msg, address );
}

void ManagementClient_ReceiveFSM::resume(JausAddress address)
{
//  pAccessControlClient_ReceiveFSM->requestAccess(address, authority);
	Resume resume_msg;
	sendJausMessage( resume_msg, address);
	queryStatus(address);
}

void ManagementClient_ReceiveFSM::set_current_client(JausAddress client)
{
	if (p_current_client != client) {
		if (p_current_client.get() != 0) {
			if (by_query) {
				p_query_timer.stop();
			} else {
				RCLCPP_INFO(logger, "cancel EVENT for mgmt status by %s", p_current_client.str().c_str());
				pEventsClient_ReceiveFSM->cancel_event(*this, p_current_client, p_query_status);
			}
			p_status = 255;
			auto ros_msg = std_msgs::msg::String();
			ros_msg.data = p_status_to_str(p_status);
			p_pub_status->publish(ros_msg);
		}
		p_current_client = client;
		if (p_current_client.get() != 0) {
			if (by_query) {
				if (p_hz > 0) {
					RCLCPP_INFO(logger, "create QUERY timer to get mgmt status from %s", p_current_client.str().c_str());
					p_query_timer.set_rate(p_hz);
					p_query_timer.start();
				} else {
					RCLCPP_WARN(logger, "invalid hz %.2f for QUERY timer to get mgmt status from %s", p_hz, p_current_client.str().c_str());
				}
			} else {
				RCLCPP_INFO(logger, "create EVENT to get mgmt status from %s", p_current_client.str().c_str());
				pEventsClient_ReceiveFSM->create_event(*this, p_current_client, p_query_status, p_hz);
			}
		}
	}
}

void ManagementClient_ReceiveFSM::delete_emergency_client()
{
	if (p_current_emergency_address.get() != 0) {
		RCLCPP_DEBUG(logger, "delete emergency client %s", p_current_client.str().c_str());
		p_current_emergency_address = JausAddress();
		pAccessControlClient_ReceiveFSM->set_emergency_client(p_current_emergency_address);
	}
}

void ManagementClient_ReceiveFSM::pQueryCallback()
{
	if (p_current_client.get() != 0) {
		sendJausMessage(p_query_status, p_current_client);
	}
}

void ManagementClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportStatus report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	reportStatusAction(report, transport_data);
}

std::string ManagementClient_ReceiveFSM::p_status_to_str(unsigned char status) {
	switch (status) {
		case 0: return "INIT";
		case 1: return "READY";
		case 2: return "STANDBY";
		case 3: return "SHUTDOWN";
		case 4: return "FAILURE";
		case 5: return "EMERGENCY";
		default: return "UNKNOWN";
	}
}

void ManagementClient_ReceiveFSM::pRosEmergency(const std_msgs::msg::Bool::SharedPtr state)
{
	if (state->data) {
		if (p_current_client.get() != 0) {
			if (p_current_emergency_address.get() != 0 && p_current_emergency_address != p_current_client) {
				RCLCPP_WARN(logger, "Something goes wrong: this client is in emergency state for %s, but controls now %s. Please release control first, clear emergency and then take control other %s to clear emergency state.",
						p_current_emergency_address.str().c_str(), p_current_client.str().c_str(), p_current_emergency_address.str().c_str());
			} else {
				p_current_emergency_address = p_current_client;
				pAccessControlClient_ReceiveFSM->set_emergency_client(p_current_client);
				SetEmergency request;
				request.getBody()->getSetEmergencyRec()->setEmergencyCode(1);
				sendJausMessage(request, p_current_client);
			}
		}
	} else if (!state->data) {
		ClearEmergency request;
		request.getBody()->getClearEmergencyRec()->setEmergencyCode(1);
		if (p_current_client.get() != 0) {
			RCLCPP_INFO(logger, "send clear emergency to current client %s", p_current_client.str().c_str());
			sendJausMessage(request, p_current_client);
		} else if (p_current_emergency_address.get() != 0) {
			RCLCPP_INFO(logger, "send clear emergency to stored address %s", p_current_emergency_address.str().c_str());
			sendJausMessage(request, p_current_emergency_address);
		}
	}
}

void ManagementClient_ReceiveFSM::pRosReady(const std_msgs::msg::Bool::SharedPtr state)
{
	if (p_current_client.get() != 0) {
		if (state->data && p_status != MANAGEMENT_STATE_READY) {
			Resume request;
			sendJausMessage(request, p_current_client);
		} else if (!state->data && p_status == MANAGEMENT_STATE_READY) {
			Standby request;
			sendJausMessage(request, p_current_client);
		}
	}
}

}
