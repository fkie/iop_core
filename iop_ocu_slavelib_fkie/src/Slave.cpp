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
	p_default_access_control = ServiceInfo::ACCESS_CONTROL_RELEASE;
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
	p_default_access_control = ServiceInfo::ACCESS_CONTROL_RELEASE;
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
			throw std::runtime_error("[ocu::Slave] no DiscoveryClient found! Please include its plugin first (in the list)!");
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
			throw std::runtime_error("[ocu::Slave] no AccessControlClient found! Please include its plugin first (in the list)!");
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
			ROS_INFO_NAMED("ocu::Slave", "no management service available! Please include its plugin first (in the list)!");
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
	service_info.update_cmd(p_default_control_addr, p_default_access_control, p_default_authority);
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
	p_pub_control_feedback = nh.advertise<iop_msgs_fkie::OcuFeedback>("ocu_feedback", 1, true);
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
	std::string access_control_str = "ACCESS_CONTROL_RELEASE(10)";
	if (p_default_access_control == 11) {
		access_control_str = "ACCESS_CONTROL_MONITOR(11)";
	} else if (p_default_access_control == 12) {
		access_control_str = "ACCESS_CONTROL_REQUEST(12)";
	}
	ROS_INFO_ONCE_NAMED("Slave", "	access_control: %s", access_control_str.c_str());
	// publish the feedback with settings
	p_sub_control = nh.subscribe<iop_msgs_fkie::OcuCmd>("ocu_cmd", 10, &Slave::pRosControl, this);
//	pFeedbackTimer = nh.createTimer(ros::Duration(15.0), &Slave::pFeedbackTimerHandler, this);
//	pFeedbackTimer.start();
}

void Slave::pRosControl(const iop_msgs_fkie::OcuCmd::ConstPtr& control)
{
	// is the command for specific client?
	for (unsigned int i = 0; i < control->cmds.size(); i++) {
		iop_msgs_fkie::OcuCmdEntry cmd = control->cmds[i];
		JausAddress ocu_client_addr = address_from_msg(cmd.ocu_client);
		JausAddress control_addr = address_from_msg(cmd.address);
		for(std::vector<ServiceInfo>::iterator it = p_services.begin(); it != p_services.end(); ++it) {
			bool apply_cmd = false;
			if (ocu_client_addr.getSubsystemID() != 0 && ocu_client_addr.getSubsystemID() != 65535) {
				if (ocu_client_addr.getSubsystemID() == it->get_own_address().getSubsystemID()) {
					if (ocu_client_addr.getNodeID() != 0 && ocu_client_addr.getNodeID() != 255) {
						if (ocu_client_addr.getNodeID() == it->get_own_address().getNodeID()) {
							if (ocu_client_addr.getComponentID() != 0 && ocu_client_addr.getComponentID() != 255) {
								if (ocu_client_addr.getComponentID() == it->get_own_address().getComponentID()) {
									apply_cmd = true;
								}
							} else {
								apply_cmd = true;
							}
						}
					} else {
						apply_cmd = true;
					}
				}
			} else {
				apply_cmd = true;
			}
			if (p_only_monitor && cmd.access_control > ServiceInfo::ACCESS_CONTROL_MONITOR) {
				apply_cmd = false;
			}
			if (p_subsystem_restricted != 65535 && control_addr.getSubsystemID() != p_subsystem_restricted) {
				apply_cmd = false;
			}
			if (apply_cmd) {
				pApplyControl(*it, control_addr, cmd.access_control, cmd.authority);
			}
		}
	}
}

void Slave::pApplyControl(ServiceInfo &service, JausAddress &control_addr, unsigned char access_control, unsigned char authority)
{
	JausAddress curr_addr = service.get_address();
	unsigned char curr_state = service.get_state();
	bool new_cmd = service.update_cmd(control_addr, access_control, authority);
	if (new_cmd) {
		switch (access_control) {
		case ServiceInfo::ACCESS_CONTROL_RELEASE:
//???					handler->access_deactivated(it->get_uri(), control_addr);
			switch (curr_state) {
//			case ServiceInfo::ACCESS_STATE_NOT_AVAILABLE:
			case ServiceInfo::ACCESS_STATE_NOT_CONTROLLED:
			case ServiceInfo::ACCESS_STATE_CONTROL_RELEASED:
			case ServiceInfo::ACCESS_STATE_MONITORING:
				service.set_state(ServiceInfo::ACCESS_STATE_NOT_CONTROLLED);
				service.handler().access_deactivated(service.get_uri(), curr_addr);
				break;
			case ServiceInfo::ACCESS_STATE_CONTROL_ACCEPTED:
				release_access(curr_addr);
				break;
			}
			break;
		case ServiceInfo::ACCESS_CONTROL_MONITOR:
			switch (curr_state) {
			case ServiceInfo::ACCESS_STATE_MONITORING:
				// already on monitoring, do nothing
				break;
			case ServiceInfo::ACCESS_STATE_CONTROL_ACCEPTED:
				// for monitoring we have to release access
				release_access(curr_addr);
				break;
			default:
				if (service.get_address().getComponentID() != 0) {
					service.set_state(ServiceInfo::ACCESS_STATE_MONITORING);
					service.handler().enable_monitoring_only(service.get_uri(), service.get_address());
				}
			}
			break;
		case ServiceInfo::ACCESS_CONTROL_REQUEST:
			switch (curr_state) {
			case ServiceInfo::ACCESS_STATE_MONITORING:
				service.set_state(ServiceInfo::ACCESS_STATE_NOT_CONTROLLED);
				service.handler().access_deactivated(service.get_uri(), curr_addr);
				// send request access
				if (service.get_address().getComponentID() != 0) {
					request_access(service.get_address(), authority);
				}
				break;
			case ServiceInfo::ACCESS_STATE_CONTROL_ACCEPTED:
				// we have already access, do nothing
				// test for other authority
				break;
			default:
				// in all other cases request control
				if (service.get_address().getComponentID() != 0) {
					request_access(service.get_address(), authority);
				}
			}
			break;
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
		pGetAccesscontrolClient()->requestAccess(address, &Slave::pAccessControlClientReplyHandler, this, authority);
	} else {
	}
}

void Slave::release_access(JausAddress &address, bool wait_for_reply)
{
	if (pGetAccesscontrolClient() != 0 && address.getComponentID() != 0) {
		if (wait_for_reply) {
			pGetAccesscontrolClient()->releaseAccess(address, &Slave::pAccessControlClientReplyHandler, this);
		} else {
			pGetAccesscontrolClient()->releaseAccess(address);
		}
	}
}

void Slave::pAccessControlClientReplyHandler(JausAddress &address, unsigned char code)
{
	iop_msgs_fkie::OcuFeedback msg_feedback;
	for(std::vector<ServiceInfo>::iterator it = p_services.begin(); it != p_services.end(); ++it) {
		if (it->get_state() != code && it->get_address().get() == address.get()) {
			switch (code) {
			case ServiceInfo::ACCESS_STATE_NOT_CONTROLLED:
			case ServiceInfo::ACCESS_STATE_CONTROL_RELEASED:
			case ServiceInfo::ACCESS_STATE_TIMEOUT:
				// access released -> stop control
				it->handler().access_deactivated(it->get_uri(), address);
				if (it->get_access_control() == ServiceInfo::ACCESS_CONTROL_MONITOR) {
					// access released, we have monitoring enabled
					it->set_state(ServiceInfo::ACCESS_STATE_MONITORING);
					it->handler().enable_monitoring_only(it->get_uri(), address);
				}
				break;
			case ServiceInfo::ACCESS_STATE_CONTROL_ACCEPTED:
				it->handler().control_allowed(it->get_uri(), address, it->get_authority());
				if (pGetManagementClient() != 0) {
					pGetManagementClient()->resume(address);
				}
				break;
			}
			it->set_state(code);
		}
	}
	// send updated info to ROS
	pSendFeedback();
}

void Slave::pFeedbackTimerHandler(const ros::TimerEvent& event)
{
	pSendFeedback();
}

void Slave::pSendFeedback()
{
	iop_msgs_fkie::OcuFeedback msg_feedback;
	for(std::vector<ServiceInfo>::iterator it = p_services.begin(); it != p_services.end(); ++it) {
		msg_feedback.services.push_back(it->to_msg());
	}
	msg_feedback.reporter = address_to_msg(p_own_address);
	msg_feedback.subsystem_restricted = p_subsystem_restricted;
	msg_feedback.only_monitor = p_only_monitor;
	p_pub_control_feedback.publish(msg_feedback);
}

void Slave::pManagementStatusHandler(JausAddress &address, unsigned char code)
{
	ROS_INFO_NAMED("ocu::Slave", "management status of %d.%d.%d changed to %d",
			address.getSubsystemID(), address.getNodeID(), address.getComponentID(), code);
}

void Slave::pDiscovered(const std::string &uri, JausAddress &address)
{
	for(std::vector<ServiceInfo>::iterator it = p_services.begin(); it != p_services.end(); ++it) {
		if (uri.compare(it->get_uri()) == 0) {
			if (it->add_discovered(address)) {
				ROS_INFO_NAMED("ocu::Slave", "Discovered '%s' at address: %d.%d.%d", uri.c_str(),
						address.getSubsystemID(), address.getNodeID(), address.getComponentID());
				//test for current control state, do we need to send access control request
				pApplyControl(*it, it->get_address(), it->get_access_control(), it->get_authority());
			}
		}
	}
}
