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
#include <urn_jaus_jss_core_AccessControlClient/AccessControlClientService.h>
#include <urn_jaus_jss_core_DiscoveryClient/DiscoveryClientService.h>
#include <urn_jaus_jss_core_ManagementClient/ManagementClientService.h>
#include <iop_msgs_fkie/JausAddress.h>
#include <iop_msgs_fkie/OcuCmd.h>
#include <iop_msgs_fkie/OcuFeedback.h>
#include <iop_ocu_slavelib_fkie/ServiceInfo.h>
#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>

namespace iop {

namespace ocu {

	class Slave
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

		bool has_access(JausAddress &address);
		void request_access(JausAddress &address, unsigned char authority);
		void release_access(JausAddress &address, bool wait_for_reply=true);
	//	JausAddress get_control_address();
	//
	//	/** Management methods **/
	//	void resume();
	//
	//	/** Set this handler to be informed if the component given @init was discovered**/
	//	template<class T>
	//	void set_control_component_handler(void(T::*handler)(unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority), T*obj);
	//	/** Set this handler be informed if accesses was granted or released. **/
	//	template<class T>
	//	void set_access_state_handler(void(T::*handler)(JausAddress &address, unsigned char value), T*obj);
	//
	  protected:
		static Slave* global_ptr;
		urn_jaus_jss_core_DiscoveryClient::DiscoveryClient_ReceiveFSM *p_discovery_client;
		urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM *p_accesscontrol_client;
		urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM *p_management_client;
		jUnsignedShortInteger p_subsystem_restricted;
		bool p_only_monitor;
		bool p_try_get_management;
		JausAddress p_own_address;
		JausAddress p_default_control_addr;
		int p_default_authority;
		int p_default_access_control;
		std::vector<ServiceInfo> p_services;
		ros::Publisher p_pub_control_feedback;
		ros::Subscriber p_sub_control;
		ros::Timer pFeedbackTimer;

		void pInitRos();
		void pRosControl(const iop_msgs_fkie::OcuCmd::ConstPtr& control);

		void pAccessControlClientReplyHandler(JausAddress &address, unsigned char code);
		void pManagementStatusHandler(JausAddress &address, unsigned char code);
		void pDiscovered(const std::string &uri, JausAddress &address);
//		void pReleaseAccess();
//		void pOcuControlPlatformHandler(unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority);
//		void pOcuControlComonentHandler(unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority);
//		void pOcuAccessControlHandler(unsigned char value);

		urn_jaus_jss_core_DiscoveryClient::DiscoveryClient_ReceiveFSM *pGetDiscoveryClient();
		urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM *pGetAccesscontrolClient();
		urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM *pGetManagementClient();
		void pApplyControl(ServiceInfo &service, JausAddress &control_addr, unsigned char access_control, unsigned char authority);
		void pFeedbackTimerHandler(const ros::TimerEvent& event);
		void pSendFeedback();

	  private:
		Slave(const Slave& other);
	};

};

};

#endif
