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

#ifndef OCUCONTROLLAYERSLAVE_H
#define OCUCONTROLLAYERSLAVE_H

#include "Transport/JausAddress.h"
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <urn_jaus_jss_core_AccessControlClient/AccessControlClientService.h>
#include <urn_jaus_jss_core_DiscoveryClient/DiscoveryClientService.h>
#include <urn_jaus_jss_core_ManagementClient/ManagementClientService.h>
#include <iop_ocu_controllib_fkie/OcuControlSlave.h>
#include <iop_msgs_fkie/JausAddress.h>
#include <iop_component_fkie/iop_component.h>

class OcuControlLayerSlave
{
  public:
	OcuControlLayerSlave();
	~OcuControlLayerSlave(void);

	void init(JausAddress own_address, std::string control_uri, jUnsignedByte major_version=1, jUnsignedByte minor_version=0);
	JausAddress get_control_address();

	/** Access Control methods **/
	bool has_access();
	void request_access();
	void release_access(bool wait_for_reply=true);

	/** Management methods **/
	void resume();

	/** This object is used be this layer. Do not change the handler! Use it carefully! **/
	OcuControlSlave &get_ocu_control_slave();

	/** Set this handler to be informed if the component given @init was discovered**/
	template<class T>
	void set_control_component_handler(void(T::*handler)(unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority), T*obj);
	/** Set this handler be informed if accesses was granted or released. **/
	template<class T>
	void set_access_state_handler(void(T::*handler)(JausAddress &address, unsigned char value), T*obj);

  protected:
	boost::function<void (unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority)> p_class_control_component_callback;
	boost::function<void (JausAddress &address, unsigned char value)> p_class_access_state_callback;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM *p_accesscontrol_client;
	urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM *p_management_client;
	urn_jaus_jss_core_DiscoveryClient::DiscoveryClient_ReceiveFSM *p_discovery_client;
	OcuControlSlave p_ocu_control_slave;
	JausAddress p_own_address;
	JausAddress p_control_address;
	std::string p_control_uri;
	jUnsignedByte p_control_uri_major_version;
	jUnsignedByte p_control_uri_minor_version;
	bool p_do_resume;

	void pAccessControlClientReplyHandler(JausAddress &address, unsigned char code);
	void pManagementStatusHandler(JausAddress &address, unsigned char code);
	void pDiscovered(const std::string &uri, JausAddress &address);
	void pReleaseAccess();
	void pOcuControlPlatformHandler(unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority);
	void pOcuControlComonentHandler(unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority);
	void pOcuAccessControlHandler(unsigned char value);
};

template<class T>
void OcuControlLayerSlave::set_control_component_handler(void(T::*handler)(unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority), T*obj) {
	p_class_control_component_callback = boost::bind(handler, obj, _1, _2, _3, _4);
	if (!p_class_control_component_callback.empty()) {
		iop_msgs_fkie::JausAddress control_component = p_ocu_control_slave.get_control_component();
		if (control_component.component_id != 0) {
			p_class_control_component_callback(control_component.subsystem_id, control_component.node_id,
					control_component.component_id, p_ocu_control_slave.get_authority());
		}
	}
}

template<class T>
void OcuControlLayerSlave::set_access_state_handler(void(T::*handler)(JausAddress &address, unsigned char value), T*obj) {
	p_class_access_state_callback = boost::bind(handler, obj, _1, _2);
	if (!p_class_access_state_callback.empty()) {
		p_class_access_state_callback(p_control_address, p_ocu_control_slave.get_access_state());
	}
}

#endif // OCUCONTROLLAYERSLAVE_H
