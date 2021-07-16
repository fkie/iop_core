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

#include <fkie_iop_ocu_slavelib/Slave.h>
#include <fkie_iop_ocu_slavelib/common.h>
#include <fkie_iop_msgs/msg/ocu_cmd_entry.hpp>
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_component/string.hpp>


using namespace urn_jaus_jss_core_AccessControlClient;
using namespace urn_jaus_jss_core_DiscoveryClient;
using namespace urn_jaus_jss_core_ManagementClient;
using namespace iop::ocu;



Slave::Slave(std::shared_ptr<iop::Component> cmp)
: logger(cmp->get_logger().get_child("Slave")),
  cmp(cmp)
{
	p_subsystem_restricted = 65535;
	p_controlled_component_nr = 1;
	p_only_monitor = false;
	p_discovery_client = NULL;
	p_accesscontrol_client = NULL;
	p_management_client = NULL;
	p_try_get_management = true;
	p_use_queries = false;
	p_default_authority = 205;
	p_default_access_control = Component::ACCESS_CONTROL_RELEASE;
	p_own_address = *(cmp->jausRouter->getJausAddress());
	p_handoff_supported = false;
	pInitRos();
}

Slave::Slave(const Slave& other)
: logger(other.logger)
{
	p_subsystem_restricted = 65535;
	p_controlled_component_nr = 1;
	p_only_monitor = false;
	p_discovery_client = NULL;
	p_accesscontrol_client = NULL;
	p_management_client = NULL;
	p_try_get_management = true;
	p_use_queries = false;
	p_default_authority = 205;
	p_default_access_control = Component::ACCESS_CONTROL_RELEASE;
	p_handoff_supported = other.p_handoff_supported;
}

Slave::~Slave(void)
{
}

DiscoveryClient_ReceiveFSM *Slave::pGetDiscoveryClient()
{
	if (p_discovery_client == NULL) {
		DiscoveryClientService *discovery_srv = static_cast<DiscoveryClientService*>(cmp->get_service("DiscoveryClientService"));
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
		AccessControlClientService *accesscontrol_srv = static_cast<AccessControlClientService*>(cmp->get_service("AccessControlClientService"));
		if (accesscontrol_srv != NULL) {
			p_accesscontrol_client = accesscontrol_srv->pAccessControlClient_ReceiveFSM;
			p_accesscontrol_client->add_reply_handler(&Slave::pAccessControlClientReplyHandler, this);
		} else {
			RCLCPP_WARN_ONCE(logger, "no AccessControlClient found! Please include its plugin first (in the list), if you needs one!");
		}
	}
	return p_accesscontrol_client;
}

ManagementClient_ReceiveFSM *Slave::pGetManagementClient()
{
	if (p_management_client == NULL && p_try_get_management) {
		p_try_get_management = false;
		ManagementClientService *management_srv = static_cast<ManagementClientService*>(cmp->get_service("ManagementClientService"));
		if (management_srv != NULL) {
			p_management_client = management_srv->pManagementClient_ReceiveFSM;
		} else {
			RCLCPP_WARN_ONCE(logger, "no management service available! Please include its plugin first (in the list), if you needs one!");
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

void Slave::set_supported_handoff(bool supported)
{
	p_handoff_supported = supported;
}

void Slave::pInitRos()
{
	RCLCPP_INFO_ONCE(logger, "=== Initialize OCU Slave ===");
	iop::Config cfg(cmp, "Slave");	
	std::string control_addr;
	cfg.declare_param<std::string>("control_addr", control_addr, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"The JAUS address if only a specific component, node or system are controlled by the client.",
		"Supported formats: XX, XX.XX or XX.XX.XX");
	cfg.declare_param<int64_t>("authority", p_default_authority, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
		"The authority for access control.",
		"Default: 205");
	cfg.declare_param<int64_t>("access_control", p_default_access_control, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
		"Access state after start the component.",
		"Default: 10; 10:ACCESS_CONTROL_RELEASE, 11:ACCESS_CONTROL_MONITOR, 12:ACCESS_CONTROL_REQUEST");
	cfg.declare_param<bool>("use_queries", p_use_queries, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
		"Enables query behaviour. Reports are send only as reply to own query.",
		"Default: false");
	cfg.declare_param<bool>("only_monitor", p_only_monitor, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
		"Disables the ability to send RequestControl and uses client only for monitoring if true",
		"Default: false");
	cfg.declare_param<int64_t>("subsystem_restricted", p_subsystem_restricted, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
		"This client can control only the specified subsystem (robot). Any by default.",
		"Default: 65535");
	cfg.declare_param<int64_t>("controlled_component", p_controlled_component_nr, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
		"Use this parameter if more than one component with requested service is available on robot. Returns the service in the list, beginning with 1.",
		"Default: 1");
	cfg.param("control_addr", control_addr, control_addr);
	if (!control_addr.empty()) {
		// try to get the address of the component specified for this slave to avoid discovering
		auto strs = iop::split(control_addr, std::string(".:"));
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
		RCLCPP_INFO_ONCE(logger, "	control_addr: %s, decoded to: %s", control_addr.c_str(), p_default_control_addr.str().c_str());
	}
	cfg.param("authority", p_default_authority, p_default_authority);
	std::map<int, std::string> access_control_map;
	access_control_map[10] = "ACCESS_CONTROL_RELEASE";
	access_control_map[11] = "ACCESS_CONTROL_MONITOR";
	access_control_map[12] = "ACCESS_CONTROL_REQUEST";
	cfg.param_named("access_control", p_default_access_control, p_default_access_control, access_control_map, true, "");
	cfg.param("use_queries", p_use_queries, p_use_queries);
	cfg.param("only_monitor", p_only_monitor, p_only_monitor);
	cfg.param("subsystem_restricted", p_subsystem_restricted, p_subsystem_restricted);
	cfg.param("controlled_component", p_controlled_component_nr, p_controlled_component_nr);
	if (p_controlled_component_nr == 0) {
		p_controlled_component_nr = 1;
	}
	p_handoff_supported = cmp->has_service("urn:jaus:jss:iop:HandoffController");
	// publish the feedback with settings
	rclcpp::QoS qos_latched(10);
	p_pub_control_feedback = cfg.create_publisher<fkie_iop_msgs::msg::OcuFeedback>("/ocu_feedback", qos_latched.transient_local());
	p_sub_control = cfg.create_subscription<fkie_iop_msgs::msg::OcuCmd>("/ocu_cmd", qos_latched.transient_local(), std::bind(&Slave::pRosControl, this, std::placeholders::_1));
}

void Slave::pRosControl(const fkie_iop_msgs::msg::OcuCmd::SharedPtr control)
{
	// is the command for specific client?
	std::map<jUnsignedInteger, std::pair<unsigned char, unsigned char> > commands;
	for (unsigned int i = 0; i < control->cmds.size(); i++) {
		auto cmd = control->cmds[i];
		JausAddress ocu_client_addr = address_from_msg(cmd.ocu_client);
		JausAddress control_addr = address_from_msg(cmd.address);
		// apply default control address
		control_addr = pApplyDefaultControlAdd(control_addr);
		if (control_addr.get() == 0) {
			continue;
		}
		// TODO: read services from discover client service
		for(std::vector<ServiceInfo>::iterator it_srv = p_services.begin(); it_srv != p_services.end(); ++it_srv) {
			bool apply_cmd = it_srv->get_own_address().match(ocu_client_addr);
			if (p_only_monitor && cmd.access_control > Component::ACCESS_CONTROL_MONITOR) {
				apply_cmd = false;
			}
			if (p_subsystem_restricted != 65535 && control_addr.getSubsystemID() != p_subsystem_restricted) {
				apply_cmd = false;
			}
			if (pGetManagementClient() != 0) {
				JausAddress emergency_addr = pGetManagementClient()->get_emergency_client();
				if (emergency_addr.get() != 0 && control_addr.get() != 0 && emergency_addr.getSubsystemID() != control_addr.getSubsystemID()) {
					apply_cmd = false;
				} else if (control_addr.get() != 0) {
					p_management_client->set_current_client(control_addr);
				}
			}
			if (apply_cmd) {
				commands[control_addr.get()] = std::make_pair(cmd.access_control, cmd.authority);
				if (cmd.access_control != Component::ACCESS_CONTROL_REQUEST) {
					// create events after request control was confirmed. The request is performed in pApplyCommands()
					pApplyToService(control_addr, cmd.access_control);
				}
			}
		}
	}
	//apply commands to each component
	pApplyCommands(commands);
}

JausAddress Slave::pApplyDefaultControlAdd(JausAddress& control_addr)
{
	JausAddress result(control_addr);
	if (p_default_control_addr.getSubsystemID() != 0 && p_default_control_addr.getSubsystemID() != 65535) {
		if (control_addr.getSubsystemID() != 0 && control_addr.getSubsystemID() != 65535) {
			if (p_default_control_addr.getSubsystemID() != control_addr.getSubsystemID()) {
				result.setSubsystemID(0);
			}
		} else {
			result.setSubsystemID(p_default_control_addr.getSubsystemID());
		}
	}
	if (p_default_control_addr.getNodeID() != 0 && p_default_control_addr.getNodeID() != 255) {
		if (control_addr.getNodeID() != 0 && control_addr.getNodeID() != 255) {
			if (p_default_control_addr.getNodeID() != control_addr.getNodeID()) {
				result.setNodeID(0);
			}
		} else {
			result.setNodeID(p_default_control_addr.getNodeID());
		}
	}
	if (p_default_control_addr.getComponentID() != 0 && p_default_control_addr.getComponentID() != 255) {
		if (control_addr.getComponentID() != 0 && control_addr.getComponentID() != 255) {
			if (p_default_control_addr.getComponentID() != control_addr.getComponentID()) {
				result.setComponentID(0);
			}
		} else {
			result.setComponentID(p_default_control_addr.getComponentID());
		}
	}
	return result;
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
					RCLCPP_DEBUG(logger, "apply command ACCESS_CONTROL_RELEASE to %d.%d.%d", (int)addr.getSubsystemID(), (int)addr.getNodeID(), (int)addr.getComponentID());
					release_access(addr);
					if (cmp->get_state() == Component::ACCESS_STATE_MONITORING) {
						cmp->set_state(Component::ACCESS_STATE_NOT_CONTROLLED);
					}
					break;
				case Component::ACCESS_CONTROL_MONITOR:
					RCLCPP_DEBUG(logger, "apply command ACCESS_CONTROL_MONITOR to %d.%d.%d", (int)addr.getSubsystemID(), (int)addr.getNodeID(), (int)addr.getComponentID());
					cmp->set_state(Component::ACCESS_STATE_MONITORING);
					break;
				case Component::ACCESS_CONTROL_REQUEST:
					RCLCPP_DEBUG(logger, "apply command ACCESS_CONTROL_REQUEST to %d.%d.%d", (int)addr.getSubsystemID(), (int)addr.getNodeID(), (int)addr.getComponentID());
					// send request access
					if (pGetAccesscontrolClient() != 0) {
						cmp->set_authority(it->second.second);
						request_access(addr, it->second.second);
					} else {
						RCLCPP_WARN(logger, "no acces control available -> set state to ACCESS_CONTROL_MONITOR to %d.%d.%d", (int)addr.getSubsystemID(), (int)addr.getNodeID(), (int)addr.getComponentID());
						cmp->set_state(Component::ACCESS_STATE_MONITORING);
					}
					break;
				}
			}
		}
	}
}

void Slave::pApplyToService(JausAddress &address, unsigned char control_state, unsigned char authority)
{
	for(std::vector<ServiceInfo>::iterator it = p_services.begin(); it != p_services.end(); ++it) {
		JausAddress discovered_addr = it->get_dicovered_address(address);
		if (discovered_addr.get() != 0) {
			switch (control_state) {
			case Component::ACCESS_CONTROL_RELEASE:
				if (it->handler().remote_addr() == discovered_addr) {
					RCLCPP_DEBUG(logger, "  inform %s about access_deactivated", it->get_uri().c_str());
					it->handler().access_deactivated(it->get_uri(), discovered_addr);
					it->set_address(discovered_addr);
					it->handler().cancel_events(it->get_uri(), discovered_addr, p_use_queries);

				}
				break;
			case Component::ACCESS_CONTROL_MONITOR:
				RCLCPP_DEBUG(logger, "  inform %s about enable_monitoring_only", it->get_uri().c_str());
				it->handler().enable_monitoring_only(it->get_uri(), discovered_addr);
				it->set_address(discovered_addr);
				it->handler().create_events(it->get_uri(), discovered_addr, p_use_queries);
				break;
			case Component::ACCESS_CONTROL_REQUEST:
				RCLCPP_INFO(logger, "  inform %s about control_allowed", it->get_uri().c_str());
				it->handler().control_allowed(it->get_uri(), discovered_addr, authority);
				it->set_address(discovered_addr);
				it->handler().create_events(it->get_uri(), discovered_addr, p_use_queries);
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
	if (pGetAccesscontrolClient() != 0 && address.get() != 0) {
		if (wait_for_reply) {
//			pGetAccesscontrolClient()->releaseAccess(address, &Slave::pAccessControlClientReplyHandler, this);
			pGetAccesscontrolClient()->releaseAccess(address);
		} else {
			pGetAccesscontrolClient()->releaseAccess(address);
		}
	}
	pApplyToService(address, Component::ACCESS_CONTROL_RELEASE);
	if (pGetManagementClient() != 0) {
		p_management_client->set_current_client(JausAddress(0));
	}
}

void Slave::pAccessControlClientReplyHandler(JausAddress &address, unsigned char code)
{
	RCLCPP_DEBUG(logger, "access control status of %s changed to %d", address.str().c_str(), code);
	auto msg_feedback = fkie_iop_msgs::msg::OcuFeedback();
	unsigned char authority = 205;
	for (unsigned int i = 0; i < p_components.size(); i++) {
		Component &cmp = p_components[i];
		if (cmp.get_address().get() == address.get()) {
			authority = cmp.get_authority();
			if (cmp.set_state(code)) {
				switch (code) {
				case Component::ACCESS_STATE_NOT_AVAILABLE:
				case Component::ACCESS_STATE_NOT_CONTROLLED:
				case Component::ACCESS_STATE_CONTROL_RELEASED:
					// access released -> stop control
					// the services are informed before release access was send
					// pApplyToService(address, Component::ACCESS_CONTROL_RELEASE);
					break;
				case Component::ACCESS_STATE_TIMEOUT:
					if (pGetManagementClient() != 0) {
						pGetManagementClient()->delete_emergency_client();
					}
					break;
				case Component::ACCESS_STATE_INSUFFICIENT_AUTHORITY:
					pApplyToService(address, Component::ACCESS_CONTROL_RELEASE);
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
		JausAddress cmp_addr = cmp.get_address();
		if (cmp_addr.match(address)) {
			return &cmp;
		}
	}
	return NULL;
}

void Slave::pSendFeedback()
{
	auto msg_feedback = fkie_iop_msgs::msg::OcuFeedback();
	for(std::vector<ServiceInfo>::iterator it = p_services.begin(); it != p_services.end(); ++it) {
		auto service_info = fkie_iop_msgs::msg::OcuServiceInfo();
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
	msg_feedback.handoff_supported = p_handoff_supported;
	p_pub_control_feedback->publish(msg_feedback);
}

void Slave::pManagementStatusHandler(JausAddress &address, unsigned char code)
{
	RCLCPP_INFO(logger, "management status of %s changed to %d", address.str().c_str(), code);
}

void Slave::pDiscovered(const std::string &uri, JausAddress &address)
{
	for(std::vector<ServiceInfo>::iterator it = p_services.begin(); it != p_services.end(); ++it) {
		if (it->add_discovered(address, uri)) {
			RCLCPP_INFO(logger, "Discovered '%s' at address: %s", uri.c_str(), address.str().c_str());
			//test for current control state, do we need to send access control request
//			pApplyControl(*it, it->get_address(), it->get_access_control(address), it->get_authority(address));
		}
	}
	pAddComponent(address);
}
