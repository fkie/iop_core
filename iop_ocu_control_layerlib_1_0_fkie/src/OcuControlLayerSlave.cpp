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

#include <iop_ocu_control_layerlib_1_0_fkie/OcuControlLayerSlave.h>

using namespace urn_jaus_jss_core_AccessControlClient_1_0;
using namespace urn_jaus_jss_core_DiscoveryClient_1_0;
using namespace urn_jaus_jss_core_ManagementClient_1_0;

OcuControlLayerSlave::OcuControlLayerSlave()
{
	p_accesscontrol_client = 0;
	p_management_client = 0;
	p_discovery_client = 0;
	p_control_uri_major_version = 1;
	p_control_uri_minor_version = 0;
	p_do_resume = false;
}

OcuControlLayerSlave::~OcuControlLayerSlave(void)
{
	release_access(false);
}

void OcuControlLayerSlave::init(JausAddress own_address, std::string control_uri, jUnsignedByte major_version, jUnsignedByte minor_version)
{
	iop::Component &cmp = iop::Component::get_instance();
	DiscoveryClientService *discovery_srv = static_cast<DiscoveryClientService*>(cmp.get_service("DiscoveryClient"));
	if (discovery_srv != NULL) {
		p_discovery_client = discovery_srv->pDiscoveryClient_ReceiveFSM;
	} else {
		throw std::runtime_error("[OcuControlLayerSlave] no DiscoveryClient found! Please include its plugin first (in the list)!");
	}
	AccessControlClientService *accesscontrol_srv = static_cast<AccessControlClientService*>(cmp.get_service("AccessControlClient"));
	if (accesscontrol_srv != NULL) {
		p_accesscontrol_client = accesscontrol_srv->pAccessControlClient_ReceiveFSM;
	} else {
		throw std::runtime_error("[OcuControlLayerSlave] no AccessControlClient found! Please include its plugin first (in the list)!");
	}
	ManagementClientService *management_srv = static_cast<ManagementClientService*>(cmp.get_service("ManagementClient"));
	if (management_srv != NULL) {
		p_management_client = management_srv->pManagementClient_ReceiveFSM;
	} else {
		ROS_INFO_NAMED("OcuControlLayerSlave", "no management service available! Please include its plugin first (in the list)!");
	}

	p_own_address = own_address;
	p_ocu_control_slave.set_reporter_address(p_own_address.getSubsystemID(), p_own_address.getNodeID(), p_own_address.getComponentID());
	p_control_uri = control_uri;
	p_control_uri_major_version = major_version;
	p_control_uri_minor_version = minor_version;

	// set callbacks
	if (p_management_client != 0) {
		p_management_client->set_status_handler(&OcuControlLayerSlave::pManagementStatusHandler, this);
	}
	p_ocu_control_slave.set_control_component_handler(&OcuControlLayerSlave::pOcuControlComonentHandler, this);
	p_ocu_control_slave.set_control_platform_handler(&OcuControlLayerSlave::pOcuControlPlatformHandler, this);
	p_ocu_control_slave.set_access_control_handler(&OcuControlLayerSlave::pOcuAccessControlHandler, this);

	// discover
//	if (p_ocu_control_slave.get_control_subsystem() == 0 && p_own_address.getSubsystemID() != 0 && !p_control_uri.empty()) {
//		ROS_INFO_NAMED("OcuControlLayerSlave", "discover %s in subsystem %d", p_control_uri.c_str(), p_own_address.getSubsystemID());
//		p_discovery_client->discover(p_control_uri, &OcuControlLayerSlave::pDiscovered, this, p_control_uri_major_version, p_control_uri_minor_version, p_own_address.getSubsystemID());
//	}
}

JausAddress OcuControlLayerSlave::get_control_address()
{
	return p_control_address;
}

bool OcuControlLayerSlave::has_access()
{
	if (p_accesscontrol_client != 0 && p_control_address.getComponentID() != 0) {
		return p_accesscontrol_client->hasAccess(p_control_address);
	}
	return false;
}

void OcuControlLayerSlave::request_access()
{
	if (p_accesscontrol_client != 0 && p_control_address.getComponentID() != 0 && p_ocu_control_slave.get_access_control() != OcuControlSlave::ACCESS_CONTROL_RELEASE) {
		p_do_resume = true;
		p_accesscontrol_client->requestAccess(p_control_address, &OcuControlLayerSlave::pAccessControlClientReplyHandler, this, p_ocu_control_slave.get_authority());
	}
}

void OcuControlLayerSlave::release_access(bool wait_for_reply)
{
	if (p_accesscontrol_client != 0 && p_control_address.getComponentID() != 0) {
		if (wait_for_reply) {
			p_accesscontrol_client->releaseAccess(p_control_address, &OcuControlLayerSlave::pAccessControlClientReplyHandler, this);
		} else {
			p_accesscontrol_client->releaseAccess(p_control_address);
		}
	}
}

void OcuControlLayerSlave::resume()
{
	if (!has_access()) {
		request_access();
	}
}

OcuControlSlave &OcuControlLayerSlave::get_ocu_control_slave()
{
	return p_ocu_control_slave;
}

void OcuControlLayerSlave::pAccessControlClientReplyHandler(JausAddress &address, unsigned char code)
{
	p_ocu_control_slave.set_access_state(code);
	if (!p_class_access_state_callback.empty()) {
		p_class_access_state_callback(address, code);
	}
	// if management client is set and resume requested
	if (code == p_accesscontrol_client->ACCESS_STATE_CONTROL_ACCEPTED) { // ACCESS_STATE_CONTROL_ACCEPTED
		if (p_do_resume && p_management_client != 0) {
			p_management_client->resume(address);
			p_do_resume = false;
		}
	}
}

void OcuControlLayerSlave::pManagementStatusHandler(JausAddress &address, unsigned char code)
{
	ROS_INFO_NAMED("OcuControlLayerSlave", "management status of %d.%d.%d changed to %d",
			address.getSubsystemID(), address.getNodeID(), address.getComponentID(), code);
}

void OcuControlLayerSlave::pDiscovered(const std::string &uri, JausAddress &address)
{
	if (uri == p_control_uri) {
		if (p_control_address.getComponentID() == 0) {
			p_control_address = address;
			p_ocu_control_slave.set_control_address(p_control_address.getSubsystemID(), p_control_address.getNodeID(), p_control_address.getComponentID());
			ROS_INFO_NAMED("OcuControlLayerSlave", "Discovered %s at address: %d.%d.%d", p_control_uri.c_str(),
					p_control_address.getSubsystemID(), p_control_address.getNodeID(), p_control_address.getComponentID());
			p_ocu_control_slave.set_control_address(p_control_address.getSubsystemID(), p_control_address.getNodeID(), p_control_address.getComponentID());
			if (!p_class_control_component_callback.empty()) {
				p_class_control_component_callback(p_control_address.getSubsystemID(), p_control_address.getNodeID(),
						p_control_address.getComponentID(), p_ocu_control_slave.get_authority());
			}
		}
	} else {
//		ROS_WARN("not expected service discovered: %s", uri.c_str());
	}
}

void OcuControlLayerSlave::pReleaseAccess()
{
	if (p_control_address.get() != 0) {
		if (p_accesscontrol_client->hasAccess(p_control_address)) {
			pAccessControlClientReplyHandler(p_control_address, p_ocu_control_slave.ACCESS_CONTROL_RELEASE);
			p_accesscontrol_client->releaseAccess(p_control_address, &OcuControlLayerSlave::pAccessControlClientReplyHandler, this);
		}
	}
}

void OcuControlLayerSlave::pOcuControlPlatformHandler(unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority)
{
	if (p_control_address.getSubsystemID() != subsystem && p_discovery_client != 0) {
		p_discovery_client->cancel_discovery(p_control_uri, p_control_uri_major_version, p_control_uri_minor_version);
		pReleaseAccess();
		p_control_address.setSubsystemID(subsystem);
		p_control_address.setNodeID(0);
		p_control_address.setComponentID(0);
		p_ocu_control_slave.set_control_address(subsystem, 0, 0);
		p_ocu_control_slave.set_access_state(p_ocu_control_slave.ACCESS_STATE_NOT_AVAILABLE);
		if (subsystem != 0 && !p_control_uri.empty()) {
			ROS_INFO_NAMED("OcuControlLayerSlave", "new subsystem to control %d -> try to discover %s...", subsystem, p_control_uri.c_str());
			p_discovery_client->discover(p_control_uri, &OcuControlLayerSlave::pDiscovered, this, p_control_uri_major_version, p_control_uri_minor_version, subsystem);
		}
	}
}

void OcuControlLayerSlave::pOcuControlComonentHandler(unsigned short subsystem, unsigned char node, unsigned char component, unsigned char authority)
{
	if (p_control_address.getSubsystemID() != subsystem
			|| p_control_address.getNodeID() != node
			|| p_control_address.getComponentID() != component) {
		p_discovery_client->cancel_discovery(p_control_uri, p_control_uri_major_version, p_control_uri_minor_version);
		pReleaseAccess();
		p_control_address.setSubsystemID(subsystem);
		p_control_address.setNodeID(node);
		p_control_address.setComponentID(component);
		p_ocu_control_slave.set_control_address(subsystem, node, component);
		p_ocu_control_slave.set_access_state(p_ocu_control_slave.ACCESS_STATE_NOT_CONTROLLED);
		ROS_INFO_NAMED("OcuControlLayerSlave", "new component to control %d.%d.%d", subsystem, node, component);
		if (!p_class_control_component_callback.empty()) {
			p_class_control_component_callback(p_control_address.getSubsystemID(), p_control_address.getNodeID(),
					p_control_address.getComponentID(), p_ocu_control_slave.get_authority());
		}
	}
}

void OcuControlLayerSlave::pOcuAccessControlHandler(unsigned char value)
{
	if (value == OcuControlSlave::ACCESS_CONTROL_ON_DEMAND) {
		ROS_INFO_NAMED("OcuControlLayerSlave", "ACCESS_CONTROL_ON_DEMAND for %s v%d.%d", p_control_uri.c_str(), p_control_uri_major_version, p_control_uri_minor_version);
	} else if (value == OcuControlSlave::ACCESS_CONTROL_REQUEST) {
		ROS_INFO_NAMED("OcuControlLayerSlave", "ACCESS_CONTROL_REQUEST for %s v%d.%d", p_control_uri.c_str(), p_control_uri_major_version, p_control_uri_minor_version);
		request_access();
	} else if (value == OcuControlSlave::ACCESS_CONTROL_RELEASE) {
		ROS_INFO_NAMED("OcuControlLayerSlave", "ACCESS_CONTROL_RELEASE for %s v%d.%d", p_control_uri.c_str(), p_control_uri_major_version, p_control_uri_minor_version);
		pReleaseAccess();
	}
}

