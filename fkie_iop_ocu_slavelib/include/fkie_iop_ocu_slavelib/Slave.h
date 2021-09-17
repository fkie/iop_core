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

#ifndef OCU_SLAVE_H
#define OCU_SLAVE_H

#include "Transport/JausAddress.h"
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <urn_jaus_jss_core_EventsClient/EventsClientService.h>
#include <urn_jaus_jss_core_AccessControlClient/Messages/MessageSet.h>
#include <urn_jaus_jss_core_AccessControlClient/AccessControlClientService.h>
#include <urn_jaus_jss_core_DiscoveryClient/DiscoveryClientService.h>
#include <urn_jaus_jss_core_ManagementClient/ManagementClientService.h>
#include <fkie_iop_msgs/JausAddress.h>
#include <fkie_iop_msgs/OcuCmd.h>
#include <fkie_iop_msgs/OcuFeedback.h>
#include <fkie_iop_ocu_slavelib/Component.h>
#include <fkie_iop_ocu_slavelib/ServiceInfo.h>
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>

namespace iop {

namespace ocu {

	class Slave: public iop::EventHandlerInterface
	{
	public:
		static Slave& get_instance(JausAddress own_address)
		{
			if (global_ptr == 0) {
				global_ptr = new Slave(own_address);
//				throw std::runtime_error("Slave was not initialized!");
			}
			return *global_ptr;
		}

		Slave(JausAddress own_address);
		~Slave(void);

		void add_supported_service(SlaveHandlerInterface &handler, std::string service_uri, jUnsignedByte major_version=1, jUnsignedByte minor_version=0);
		void set_supported_handoff(bool supported);

		bool has_access(JausAddress &address);
		void request_access(JausAddress &address, unsigned char authority);
		void release_access(JausAddress &address, bool wait_for_reply=true);

		/// EventHandlerInterface Methods
		void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	protected:
		static Slave* global_ptr;
		urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM *p_events_client;
		urn_jaus_jss_core_DiscoveryClient::DiscoveryClient_ReceiveFSM *p_discovery_client;
		urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM *p_accesscontrol_client;
		urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM *p_management_client;
		int p_subsystem_restricted;
		int p_controlled_component_nr;
		bool p_only_monitor;
		bool p_try_get_management;
		bool p_use_queries;
		bool p_handoff_supported;
		JausAddress p_own_address;
		JausAddress p_default_control_addr;
		JausAddress p_current_control_addr;
		unsigned char p_current_control_state;
		int p_default_authority;
		int p_default_access_control;
		std::vector<ServiceInfo> p_services;
		std::vector<Component> p_components;
		std::vector<JausAddress> p_access_control_addresses;
		ros::Publisher p_pub_control_feedback;
		ros::Publisher p_pub_ac_reports;
		ros::Subscriber p_sub_control;
		ros::Timer pFeedbackTimer;
		urn_jaus_jss_core_AccessControlClient::QueryControl p_query_control;

		void pInitRos();
		void pRosControl(const fkie_iop_msgs::OcuCmd::ConstPtr& control);

		void pAccessControlClientReplyHandler(JausAddress &address, unsigned char code);
		void pManagementStatusHandler(JausAddress &address, unsigned char code);
		void pDiscovered(const std::string &uri, JausAddress &address);

		urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM *pGetEventsClient();
		urn_jaus_jss_core_DiscoveryClient::DiscoveryClient_ReceiveFSM *pGetDiscoveryClient();
		urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM *pGetAccesscontrolClient();
		urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM *pGetManagementClient();
		void pAddComponent(JausAddress &address);
		Component* pGetComponent(JausAddress &address);
		JausAddress pApplyDefaultControlAdd(JausAddress& control_addr);
		void pApplyCommands(std::map<jUnsignedInteger, std::pair<unsigned char, unsigned char> > commands);
		void pApplyToService(JausAddress &addr, unsigned char control_state, unsigned char authority=205);
		void pApplyControl(ServiceInfo &service, JausAddress &control_addr, unsigned char access_control, unsigned char authority);
		void pSendFeedback();

	private:
		Slave(const Slave& other);
	};

};

};

#endif
