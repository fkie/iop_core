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

#include <boost/algorithm/string.hpp>
#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_ocu_slavelib_fkie/common.h>
#include <iop_component_fkie/iop_component.h>
#include <iop_msgs_fkie/OcuCmdEntry.h>


using namespace urn_jaus_jss_core_AccessControlClient;
using namespace urn_jaus_jss_core_DiscoveryClient;
using namespace urn_jaus_jss_core_ManagementClient;
using namespace iop::ocu;

Slave* Slave::global_ptr = 0;


Slave::Slave(JausAddress own_address)
{
	global_ptr = this;
	p_subsystem_restricted = 65535;
	p_only_monitor = false;
	p_discovery_client = NULL;
	p_accesscontrol_client = NULL;
	p_management_client = NULL;
	p_try_get_management = true;
	p_default_authority = 205;
	p_default_access_control = Component::ACCESS_CONTROL_RELEASE;
	p_own_address = own_address;
	pInitRos();
}

Slave::Slave(const Slave& other)
{
	p_subsystem_restricted = 65535;
	p_only_monitor = false;
	p_discovery_client = NULL;
	p_accesscontrol_client = NULL;
	p_management_client = NULL;
	p_try_get_management = true;
	p_default_authority = 205;
	p_default_access_control = Component::ACCESS_CONTROL_RELEASE;
}

Slave::~Slave(void)
{
	pFeedbackTimer.stop();
	delete global_ptr;
}

DiscoveryClient_ReceiveFSM *Slave::pGetDiscoveryClient()
{
	if (p_discovery_client == NULL) {
		iop::Component &cmp = iop::Component::get_instance();
		DiscoveryClientService *discovery_srv = static_cast<DiscoveryClientService*>(cmp.get_service("DiscoveryClient"));
		if (discovery_srv != NULL) {
			p_discovery_client = discovery_srv->pDiscoveryClient_ReceiveFSM;
		} else {
			throw std::runtime_error("[Slave] no DiscoveryClient found! Please include its plugin first (in the list)!");
		}
	}
	return p_discovery_client;
}

AccessControlClient_ReceiveFSM *Slave::pGetAccesscontrolClient()
{
	if (p_accesscontrol_client == NULL) {
		iop::Component &cmp = iop::Component::get_instance();
		AccessControlClientService *accesscontrol_srv = static_cast<AccessControlClientService*>(cmp.get_service("AccessControlClient"));
		if (accesscontrol_srv != NULL) {
			p_accesscontrol_client = accesscontrol_srv->pAccessControlClient_ReceiveFSM;
			p_accesscontrol_client->add_reply_handler(&Slave::pAccessControlClientReplyHandler, this);
		} else {
			ROS_WARN_ONCE_NAMED("Slave", "no AccessControlClient found! Please include its plugin first (in the list), if you needs one!");
		}
	}
	return p_accesscontrol_client;
}

ManagementClient_ReceiveFSM *Slave::pGetManagementClient()
{
	if (p_management_client == NULL && p_try_get_management) {
		p_try_get_management = false;
		iop::Component &cmp = iop::Component::get_instance();
		ManagementClientService *management_srv = static_cast<ManagementClientService*>(cmp.get_service("ManagementClient"));
		if (management_srv != NULL) {
			p_management_client = management_srv->pManagementClient_ReceiveFSM;
		} else {
			ROS_WARN_ONCE_NAMED("Slave", "no management service available! Please include its plugin first (in the list), if you needs one!");
		}
		// set callbacks
		if (p_management_client != 0) {
			p_management_client->set_status_handler(&Slave::pManagementStatusHandler, this);
		}
	}
	return p_management_client;
}

void Slave::add_supported_service(SlaveHandlerInterface &handler, std::string service_uri, jUnsignedByte major_version, jUnsignedByte minor_version)
{
	ServiceInfo service_info(handler, p_own_address, service_uri, major_version, minor_version);
	p_services.push_back(service_info);
	// start discovering
	if (pGetDiscoveryClient() != NULL) {
		pGetDiscoveryClient()->discover(service_uri, &Slave::pDiscovered, this, major_version, minor_version);
	}
	pSendFeedback();
}

void Slave::pInitRos()
{
	ros::NodeHandle nh;
	p_pub_control_feedback = nh.advertise<iop_msgs_fkie::OcuFeedback>("/ocu_feedback", 1, true);
	ros::NodeHandle pnh("~");
	std::string control_addr;
	if (!pnh.getParam("control_addr", control_addr)) {
		nh.param("control_addr", control_addr, control_addr);
	}
	if (!pnh.getParam("authority", p_default_authority)) {
		nh.param("authority", p_default_authority, p_default_authority);
	}
	if (!pnh.getParam("access_control", p_default_access_control)) {
		nh.param("access_control", p_default_access_control, p_default_access_control);
	}
	if (!pnh.getParam("only_monitor", p_only_monitor)) {
		nh.param("only_monitor", p_only_monitor, p_only_monitor);
	}
	if (!pnh.getParam("subsystem_restricted", p_subsystem_restricted)) {
		nh.param("subsystem_restricted", p_subsystem_restricted, p_subsystem_restricted);
	}
	if (!control_addr.empty()) {
		// try to get the address of the component specified for this slave to avoid discovering
		std::vector<std::string> strs;
		boost::split(strs, control_addr, boost::is_any_of(".:"));
		if (strs.size() > 3) {
			throw new std::string("Invalid control_addr parameter: "+ control_addr + "\n");
		} else {
			if (strs.size() > 0) {
				p_default_control_addr.setSubsystemID(atoi(strs[0].c_str()));
			}
			if (strs.size() > 1) {
				p_default_control_addr.setNodeID(atoi(strs[1].c_str()));
			}
			if (strs.size() > 2) {
				p_default_control_addr.setComponentID(atoi(strs[2].c_str()));
			}
		}
	}
	ROS_INFO_ONCE_NAMED("Slave", "OCU control slave parameter:");
	ROS_INFO_ONCE_NAMED("Slave", "	control_addr: %s, decoded to: %d.%d.%d", control_addr.c_str(),
			p_default_control_addr.getSubsystemID(), p_default_control_addr.getNodeID(), p_default_control_addr.getComponentID());
	ROS_INFO_ONCE_NAMED("Slave", "	authority:	%d", p_default_authority);
	ROS_INFO_ONCE_NAMED("Slave", "	only_monitor:	%d", (int)p_only_monitor);
	ROS_INFO_ONCE_NAMED("Slave", "	subsystem_restricted:	%d", p_subsystem_restricted);
	std::string access_control_str = "ACCESS_CONTROL_RELEASE(10)";
	if (p_default_access_control == 11) {
		access_control_str = "ACCESS_CONTROL_MONITOR(11)";
	} else if (p_default_access_control == 12) {
		access_control_str = "ACCESS_CONTROL_REQUEST(12)";
	}
	ROS_INFO_ONCE_NAMED("Slave", "	access_control: %s", access_control_str.c_str());
	// publish the feedback with settings
	p_sub_control = nh.subscribe<iop_msgs_fkie::OcuCmd>("/ocu_cmd", 10, &Slave::pRosControl, this);
}

void Slave::pRosControl(const iop_msgs_fkie::OcuCmd::ConstPtr& control)
{
	// is the command for specific client?
	std::map<jUnsignedInteger, std::pair<unsigned char, unsigned char> > commands;
	for (unsigned int i = 0; i < control->cmds.size(); i++) {
		iop_msgs_fkie::OcuCmdEntry cmd = control->cmds[i];
		JausAddress ocu_client_addr = address_from_msg(cmd.ocu_client);
		JausAddress control_addr = address_from_msg(cmd.address);
		for(std::vector<ServiceInfo>::iterator it = p_services.begin(); it != p_services.end(); ++it) {
			bool apply_cmd = match_address(it->get_own_address(), ocu_client_addr);
			if (p_only_monitor && cmd.access_control > Component::ACCESS_CONTROL_MONITOR) {
				apply_cmd = false;
			}
			if (p_subsystem_restricted != 65535 && control_addr.getSubsystemID() != p_subsystem_restricted) {
				apply_cmd = false;
			}
			if (apply_cmd) {
				for (std::vector<Component>::iterator it = p_components.begin(); it != p_components.end(); ++it) {
					if (it->match(control_addr)) {
						commands[it->get_address().get()] = std::make_pair(cmd.access_control, cmd.authority);
					}
				}
			}
		}
	}
	//apply commands to each component
	pApplyCommands(commands);
}

void Slave::pApplyCommands(std::map<jUnsignedInteger, std::pair<unsigned char, unsigned char> > commands)
{
	std::map<jUnsignedInteger, std::pair<unsigned char, unsigned char> >::iterator it;
	for (it = commands.begin(); it != commands.end(); ++it) {
		JausAddress addr(it->first);
		Component* cmp = pGetComponent(addr);
		if (cmp != NULL) {
			// it is new control for the component or new authority
			if (cmp->set_access_control(it->second.first) or cmp->set_authority(it->second.second)) {
				switch (it->second.first) {
				case Component::ACCESS_CONTROL_RELEASE:
					ROS_DEBUG_NAMED("Slave", "apply command ACCESS_CONTROL_RELEASE to %d.%d.%d", (int)addr.getSubsystemID(), (int)addr.getNodeID(), (int)addr.getComponentID());
					pApplyToService(addr, it->second.first);
					release_access(addr);
					if (cmp->get_state() == Component::ACCESS_STATE_MONITORING) {
						cmp->set_state(Component::ACCESS_STATE_NOT_CONTROLLED);
					}
					break;
				case Component::ACCESS_CONTROL_MONITOR:
					ROS_DEBUG_NAMED("Slave", "apply command ACCESS_CONTROL_MONITOR to %d.%d.%d", (int)addr.getSubsystemID(), (int)addr.getNodeID(), (int)addr.getComponentID());
					cmp->set_state(Component::ACCESS_STATE_MONITORING);
					pApplyToService(addr, it->second.first);
					break;
				case Component::ACCESS_CONTROL_REQUEST:
					ROS_DEBUG_NAMED("Slave", "apply command ACCESS_CONTROL_REQUEST to %d.%d.%d", (int)addr.getSubsystemID(), (int)addr.getNodeID(), (int)addr.getComponentID());
					// send request access
					cmp->set_authority(it->second.second);
					request_access(addr, it->second.second);
					break;
				}
			}
		}
	}

}

void Slave::pApplyToService(JausAddress &address, unsigned char control_state, unsigned char authority)
{
	for(std::vector<ServiceInfo>::iterator it = p_services.begin(); it != p_services.end(); ++it) {
		if (it->has_component(address)) {
			switch (control_state) {
			case Component::ACCESS_CONTROL_RELEASE:
				ROS_DEBUG_NAMED("Slave", "  inform %s about access_deactivated", it->get_uri().c_str());
				it->handler().access_deactivated(it->get_uri(), address);
				it->set_address(address);
				break;
			case Component::ACCESS_CONTROL_MONITOR:
				ROS_DEBUG_NAMED("Slave", "  inform %s about enable_monitoring_only", it->get_uri().c_str());
				it->handler().enable_monitoring_only(it->get_uri(), address);
				it->set_address(address);
				break;
			case Component::ACCESS_CONTROL_REQUEST:
				ROS_DEBUG_NAMED("Slave", "  inform %s about control_allowed", it->get_uri().c_str());
				it->handler().control_allowed(it->get_uri(), address, authority);
				it->set_address(address);
				break;
			}
		}
	}
}

bool Slave::has_access(JausAddress &address)
{
	if (pGetAccesscontrolClient() != 0) {
		return pGetAccesscontrolClient()->hasAccess(address);
	}
	return false;
}

void Slave::request_access(JausAddress &address, unsigned char authority)
{
	if (pGetAccesscontrolClient() != 0 && address.get() != 0) {
//		pGetAccesscontrolClient()->requestAccess(address, &Slave::pAccessControlClientReplyHandler, this, authority);
		pGetAccesscontrolClient()->requestAccess(address, authority);
	} else {
	}
}

void Slave::release_access(JausAddress &address, bool wait_for_reply)
{
	if (pGetAccesscontrolClient() != 0 && address.getComponentID() != 0) {
		if (wait_for_reply) {
//			pGetAccesscontrolClient()->releaseAccess(address, &Slave::pAccessControlClientReplyHandler, this);
			pGetAccesscontrolClient()->releaseAccess(address);
		} else {
			pGetAccesscontrolClient()->releaseAccess(address);
		}
	}
}

void Slave::pAccessControlClientReplyHandler(JausAddress &address, unsigned char code)
{
	iop_msgs_fkie::OcuFeedback msg_feedback;
	Component* cmp = pGetComponent(address);
	unsigned char authority = 205;
	if (cmp != NULL) {
		authority = cmp->get_authority();
		if (cmp->set_state(code)) {
			switch (code) {
			case Component::ACCESS_STATE_NOT_CONTROLLED:
			case Component::ACCESS_STATE_CONTROL_RELEASED:
			case Component::ACCESS_STATE_TIMEOUT:
				// access released -> stop control
				// the services are informed before release access was send
				// pApplyToService(address, Component::ACCESS_CONTROL_RELEASE);
				break;
			case Component::ACCESS_STATE_CONTROL_ACCEPTED:
				// pass authority to the handler
				pApplyToService(address, Component::ACCESS_CONTROL_REQUEST, authority);
				if (pGetManagementClient() != 0) {
					pGetManagementClient()->resume(address);
				}
				break;
			}
			// send updated info to ROS
			pSendFeedback();
		}
	}
}

void Slave::pAddComponent(JausAddress &address)
{
	for (std::vector<Component>::iterator it = p_components.begin(); it != p_components.end(); ++it) {
		if (it->get_address().get() == address.get()) {
			return;
		}
	}
	Component cmp(address);
	cmp.set_access_control(p_default_access_control);
	cmp.set_authority(p_default_authority);
	p_components.push_back(cmp);
}

Component* Slave::pGetComponent(JausAddress &address)
{

	for (unsigned int i = 0; i < p_components.size(); i++) {
		Component &cmp = p_components[i];
		if (cmp.get_address().get() == address.get()) {
			return &cmp;
		}
	}
	return NULL;
}

void Slave::pSendFeedback()
{
	iop_msgs_fkie::OcuFeedback msg_feedback;
	for(std::vector<ServiceInfo>::iterator it = p_services.begin(); it != p_services.end(); ++it) {
		iop_msgs_fkie::OcuServiceInfo service_info;
		service_info.uri = it->get_uri();
		service_info.addr_control = address_to_msg(it->get_address());
		Component* cmp = pGetComponent(it->get_address());
		if (cmp != NULL) {
			service_info.access_state = cmp->get_state();
			service_info.authority = cmp->get_authority();
		}
		msg_feedback.services.push_back(service_info);
	}
	msg_feedback.reporter = address_to_msg(p_own_address);
	msg_feedback.subsystem_restricted = p_subsystem_restricted;
	msg_feedback.only_monitor = p_only_monitor;
	p_pub_control_feedback.publish(msg_feedback);
}

void Slave::pManagementStatusHandler(JausAddress &address, unsigned char code)
{
	ROS_INFO_NAMED("Slave", "management status of %d.%d.%d changed to %d",
			address.getSubsystemID(), address.getNodeID(), address.getComponentID(), code);
}

void Slave::pDiscovered(const std::string &uri, JausAddress &address)
{
	for(std::vector<ServiceInfo>::iterator it = p_services.begin(); it != p_services.end(); ++it) {
		if (uri.compare(it->get_uri()) == 0) {
			if (it->add_discovered(address)) {
				ROS_INFO_NAMED("Slave", "Discovered '%s' at address: %d.%d.%d", uri.c_str(),
						address.getSubsystemID(), address.getNodeID(), address.getComponentID());
				//test for current control state, do we need to send access control request
//				pApplyControl(*it, it->get_address(), it->get_access_control(address), it->get_authority(address));
			}
		}
	}
	pAddComponent(address);
}
