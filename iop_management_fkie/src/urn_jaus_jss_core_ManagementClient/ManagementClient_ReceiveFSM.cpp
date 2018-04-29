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
#include <iop_component_fkie/iop_component.h>


#include <ros/ros.h>
#include <ros/console.h>

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

ManagementClient_ReceiveFSM::ManagementClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new ManagementClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	p_status = 255;
	p_hz = 1.0;
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

	iop::Config cfg("~ManagementClient");
	cfg.param("hz", p_hz, p_hz, false, false);
	cfg.param("by_query", by_query, by_query, true, true);
	p_pub_status = cfg.advertise<std_msgs::String>("mgmt_status", 5, true);
	p_sub_cmd_emergency = cfg.subscribe<std_msgs::Bool>("cmd_mgmt_emergency", 5, &ManagementClient_ReceiveFSM::pRosEmergency, this);
	p_sub_cmd_ready = cfg.subscribe<std_msgs::Bool>("cmd_mgmt_reset", 5, &ManagementClient_ReceiveFSM::pRosReady, this);
}

void ManagementClient_ReceiveFSM::reportStatusAction(ReportStatus msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	jUnsignedByte new_state = msg.getBody()->getReportStatusRec()->getStatus();
	if (p_status != new_state) {
		p_status = new_state;
		ROS_DEBUG_NAMED("ManagementClient", "new management status: %d (%s)", p_status, p_status_to_str(p_status).c_str());
		std_msgs::String ros_msg;
		ros_msg.data = p_status_to_str(p_status);
		p_pub_status.publish(ros_msg);
		//    p_on_status_query = false;
		//    if (p_do_resume and p_status != 3) {
		//      ROS_DEBUG_NAMED("ManagementClient", "  Resume!\n");
		//      p_do_resume = false;
		//      queryStatus(sender);
		//    }
		if (!p_class_interface_callback.empty() && p_current_client == sender) {
			ROS_DEBUG_NAMED("ManagementClient", "  forward management state to handler");
			p_class_interface_callback(sender, p_status);
		}
		if (p_status == 2 && p_current_client == sender) {
			if (pAccessControlClient_ReceiveFSM->hasAccess(p_current_client )) {
				resume(p_current_client);
			}
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
				ROS_INFO_NAMED("ManagementClient", "cancel EVENT for mgmt status by %s", p_current_client.str().c_str());
				pEventsClient_ReceiveFSM->cancel_event(*this, p_current_client, p_query_status);
			}
			p_status = 255;
			std_msgs::String ros_msg;
			ros_msg.data = p_status_to_str(p_status);
			p_pub_status.publish(ros_msg);
		}
		p_current_client = client;
		if (p_current_client.get() != 0) {
			if (by_query) {
				if (p_hz > 0) {
					ROS_INFO_NAMED("ManagementClient", "create QUERY timer to get mgmt status from %s", p_current_client.str().c_str());
					p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &ManagementClient_ReceiveFSM::pQueryCallback, this);
				} else {
					ROS_WARN_NAMED("ManagementClient", "invalid hz %.2f for QUERY timer to get mgmt status from %s", p_hz, p_current_client.str().c_str());
				}
			} else {
				ROS_INFO_NAMED("ManagementClient", "create EVENT to get mgmt status from %s", p_current_client.str().c_str());
				pEventsClient_ReceiveFSM->create_event(*this, p_current_client, p_query_status, 0);
			}
		}
	}
}

void ManagementClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
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

void ManagementClient_ReceiveFSM::pRosEmergency(const std_msgs::Bool::ConstPtr& state)
{
	if (p_current_client.get() != 0) {
		if (state->data && p_status != MANAGEMENT_STATE_EMERGENCY) {
			SetEmergency request;
			request.getBody()->getSetEmergencyRec()->setEmergencyCode(1);
			sendJausMessage(request, p_current_client);
		} else if (!state->data && p_status == MANAGEMENT_STATE_EMERGENCY) {
			ClearEmergency request;
			request.getBody()->getClearEmergencyRec()->setEmergencyCode(1);
			sendJausMessage(request, p_current_client);
		}
	}
}

void ManagementClient_ReceiveFSM::pRosReady(const std_msgs::Bool::ConstPtr& state)
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

};
