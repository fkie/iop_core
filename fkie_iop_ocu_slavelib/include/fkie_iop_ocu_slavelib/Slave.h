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

#include <rclcpp/rclcpp.hpp>
#include "Transport/JausAddress.h"
#include <urn_jaus_jss_core_AccessControlClient/AccessControlClientService.h>
#include <urn_jaus_jss_core_DiscoveryClient/DiscoveryClientService.h>
#include <urn_jaus_jss_core_ManagementClient/ManagementClientService.h>
#include <fkie_iop_component/iop_component.hpp>
#include <fkie_iop_msgs/msg/jaus_address.hpp>
#include <fkie_iop_msgs/msg/ocu_cmd.hpp>
#include <fkie_iop_msgs/msg/ocu_feedback.hpp>
#include "Component.h"
#include "ServiceInfo.h"
#include "SlaveHandlerInterface.h"

namespace iop {

namespace ocu {

	class Slave
	{
	public:
		static std::shared_ptr<iop::ocu::Slave> get_instance(std::shared_ptr<iop::Component> cmp)
		{
			if (cmp->get_slave().get() == nullptr) {
				cmp->set_slave(std::make_shared<Slave>(cmp));
			}
			return cmp->get_slave();
		}

		Slave(std::shared_ptr<iop::Component> cmp);
		~Slave(void);

		void add_supported_service(SlaveHandlerInterface &handler, std::string service_uri, jUnsignedByte major_version=1, jUnsignedByte minor_version=0);
		void set_supported_handoff(bool supported);

		bool has_access(JausAddress &address);
		void request_access(JausAddress &address, unsigned char authority);
		void release_access(JausAddress &address, bool wait_for_reply=true);

	protected:
		urn_jaus_jss_core_DiscoveryClient::DiscoveryClient_ReceiveFSM *p_discovery_client;
		urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM *p_accesscontrol_client;
		urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM *p_management_client;
		rclcpp::Logger logger;
		std::shared_ptr<iop::Component> cmp;
		int p_subsystem_restricted;
		int p_controlled_component_nr;
		bool p_only_monitor;
		bool p_try_get_management;
		bool p_use_queries;
		bool p_handoff_supported;
		JausAddress p_own_address;
		JausAddress p_default_control_addr;
		int p_default_authority;
		int p_default_access_control;
		std::vector<ServiceInfo> p_services;
		std::vector<Component> p_components;
		rclcpp::Publisher<fkie_iop_msgs::msg::OcuFeedback>::SharedPtr p_pub_control_feedback;
		rclcpp::Subscription<fkie_iop_msgs::msg::OcuCmd>::SharedPtr p_sub_control;

		void pInitRos();
		void pRosControl(const fkie_iop_msgs::msg::OcuCmd::SharedPtr control);

		void pAccessControlClientReplyHandler(JausAddress &address, unsigned char code);
		void pManagementStatusHandler(JausAddress &address, unsigned char code);
		void pDiscovered(const std::string &uri, JausAddress &address);

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

}

}

#endif
