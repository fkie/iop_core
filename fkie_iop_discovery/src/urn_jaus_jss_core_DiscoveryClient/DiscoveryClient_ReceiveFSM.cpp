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


#include "urn_jaus_jss_core_DiscoveryClient/DiscoveryClient_ReceiveFSM.h"

#include <algorithm>
#include <fkie_iop_component/iop_component.h>
#include <fkie_iop_component/iop_config.h>
#include <fkie_iop_component/ros_node.hpp>
#include <fkie_iop_component/time.hpp>


using namespace JTS;
using namespace urn_jaus_jss_core_Discovery;

namespace urn_jaus_jss_core_DiscoveryClient
{

DiscoveryClient_ReceiveFSM::DiscoveryClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM)
: logger(iop::RosNode::get_instance().get_logger().get_child("DiscoveryClient")),
  p_timer(std::chrono::seconds(15), std::bind(&DiscoveryClient_ReceiveFSM::pTimeoutCallback, this), false)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new DiscoveryClient_ReceiveFSMContext(*this);
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	system_id = 4;
	register_own_services = true;
	TIMEOUT_DISCOVER = 5;
	TIMEOUT_STANDBY = 15;
	p_current_timeout = TIMEOUT_STANDBY;
	p_discovery_fsm = NULL;
	p_first_ready = true;
	p_is_registered = false;
	p_on_registration = false;
	p_count_discover_tries = 0;
	p_timeout_event = new InternalEvent("Timeout", "ControlTimeout");
	p_current_diagnostic_level = 3;
	p_timeout_discover_service = 60;
}



DiscoveryClient_ReceiveFSM::~DiscoveryClient_ReceiveFSM()
{

	p_timer.stop();
	delete context;
	delete p_timeout_event;
}

void DiscoveryClient_ReceiveFSM::setupNotifications()
{
	pEventsClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_DiscoveryClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	pEventsClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_DiscoveryClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving_Ready", "DiscoveryClient_ReceiveFSM");
	registerNotification("Receiving", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving", "DiscoveryClient_ReceiveFSM");
	iop::Config cfg("DiscoveryClient");
	cfg.param("system_id", system_id, system_id, true, "", p_system_id_map());
	cfg.param("register_own_services", register_own_services, register_own_services, true);
	cfg.param("timeout_discover_service", p_timeout_discover_service, p_timeout_discover_service);
	cfg.param("query_timeout_discover", TIMEOUT_DISCOVER, TIMEOUT_DISCOVER);
	if (TIMEOUT_DISCOVER == 0) {
		RCLCPP_WARN(logger, "query_timeout_discover is configured to zero, increase to 5");
		TIMEOUT_DISCOVER = 5;
	}
	cfg.param("query_timeout_standby", TIMEOUT_STANDBY, TIMEOUT_STANDBY);
	if (TIMEOUT_STANDBY == 0) {
		RCLCPP_WARN(logger, "query_timeout_standby is configured to zero, increase to 15");
		TIMEOUT_STANDBY = 15;
	}
	p_ros_interface.setup(*this);
	p_ros_interface.set_discovery_timeout(p_timeout_discover_service);
	std::vector<std::string> v;
	cfg.param("unicast_subsystems", v, v);
	for(unsigned int i = 0; i < v.size(); i++) {
		std::string jaus_addr_str = v[i];
		// parsce jaus address
		int p1, p2, p3;
		int scan_result = std::sscanf(jaus_addr_str.c_str(), "%d.%d.%d", &p1, &p2, &p3);
		if (scan_result == 3) {
			JausAddress jaus_addr(p1, p2, p3);
			RCLCPP_WARN(logger, "unicast `discovery` for %s. Please add `J%d = \"IPv4:3497\"` to AddressBook in JAUS configuration file. If already done, you can ignore this warning.", jaus_addr.str().c_str(), jaus_addr.get());
			p_unicast_subsystems.push_back(jaus_addr);
		} else {
			RCLCPP_WARN(logger, "invalid address format in unicast_subsystems: %s, should be subsystem.node.component", jaus_addr_str.c_str());
		}
	}
	// } else {
	// 	RCLCPP_WARN(logger, "wrong ~unicast_subsystems parameter type! It should be an array with format [123.45.67, ...]");
	// }
	p_timer.set_interval(std::chrono::seconds(p_current_timeout));
	p_timer.start();
	if (!register_own_services) {
		pRegistrationFinished();
	}
	if (!p_is_registered) {
		dynamic_cast<iop::IopJausRouter*>(jausRouter)->getComponent()->send_diagnostic(3, "Register services");
	}
}

std::map<int, std::string> DiscoveryClient_ReceiveFSM::p_system_id_map()
{
	std::map<int, std::string> result;
	result[1] = "System";
	result[2] = "Subsystem";
	result[3] = "Node";
	result[4] = "Component";
	return result;
}

void DiscoveryClient_ReceiveFSM::setDiscoveryFSM(Discovery_ReceiveFSM *discovery_fsm)
{
	if (discovery_fsm != NULL) {
		if (discovery_fsm->getSystemID() == TYPE_SUBSYSTEM) {
			// only uses if it is defined as SUBSYSTEM
			p_discovery_fsm = discovery_fsm;
			for (unsigned int i = 0; i < p_own_uri_services.size(); i++) {
				iop::DiscoveryServiceDef &service = p_own_uri_services[i];
				p_discovery_fsm->registerService(service.minor_version,
						service.major_version,
						service.service_uri,
						*jausRouter->getJausAddress());
			}
			pRegistrationFinished();
		}
	}
}

void DiscoveryClient_ReceiveFSM::handleQueryIdentificationAction(QueryIdentification msg, Receive::Body::ReceiveRec transportData)
{
	if (p_discovery_fsm != NULL) {
		throw std::runtime_error("we have a discovery service, it should respond");
		RCLCPP_WARN(logger, "QueryIdentification received although discovery service defined!");
		// TODO forward to discovery service
		//p_discovery_fsm->sendReportIdentificationAction(msg, transportData);
	} else if (register_own_services) {
		// reply with component name
		int query_type = msg.getBody()->getQueryIdentificationRec()->getQueryType();
		if (query_type == TYPE_COMPONENT) { // COMPONENT
			JausAddress sender = transportData.getAddress();
			ReportIdentification report_msg;
			std::string name = iop::RosNode::get_instance().get_name();
			std::size_t pos = name.find_last_of("/");
			if (pos != std::string::npos) {
				name.replace(0, pos+1, "");
			}
			int system_type = 60001;  // COMPONENT
			RCLCPP_DEBUG(logger, "sendReportIdentification to %s: query_type: %d, system_type: %d, name: %s",
					sender.str().c_str(),query_type, system_type, name.c_str());
			report_msg.getBody()->getReportIdentificationRec()->setQueryType(query_type);
			report_msg.getBody()->getReportIdentificationRec()->setType(system_type);
			report_msg.getBody()->getReportIdentificationRec()->setIdentification(name);
			sendJausMessage(report_msg, sender);
		}
	}
}

void DiscoveryClient_ReceiveFSM::pRegistrationFinished()
{
	p_is_registered = true;
	RCLCPP_INFO(logger, "Service registration by discovery service finished!");
	p_current_diagnostic_level = 0;
	dynamic_cast<iop::IopJausRouter*>(jausRouter)->getComponent()->send_diagnostic(0, "Registration finished");
	pCheckTimer();
}

void DiscoveryClient_ReceiveFSM::pCheckTimer()
{
	int64_t timeoutts = TIMEOUT_STANDBY;
	bool in_discover = false;
	if (!p_is_registered || pHasToDiscover(65535)) {
		timeoutts = TIMEOUT_DISCOVER;
		if (p_count_discover_tries > TIMEOUT_STANDBY / TIMEOUT_DISCOVER) {
			if (timeoutts != TIMEOUT_STANDBY) {
				RCLCPP_DEBUG(logger, "max tries for discovery services reached, increase timeout");
				timeoutts = TIMEOUT_STANDBY;
				p_current_diagnostic_level = 1;
				dynamic_cast<iop::IopJausRouter*>(jausRouter)->getComponent()->send_diagnostic(1, "Not all services discovered");
			}
		}
		in_discover = true;
	}

	if (p_current_timeout != timeoutts) {
		if (!in_discover) {
			p_current_diagnostic_level = 0;
			dynamic_cast<iop::IopJausRouter*>(jausRouter)->getComponent()->send_diagnostic(0, "All services discovered");
		}
		if (p_current_timeout > timeoutts) {
			RCLCPP_INFO(logger, "reduce timeout to %d sec", timeoutts);
		} else {
			RCLCPP_INFO(logger, "increase timeout to %d sec", timeoutts);
		}
		p_current_timeout = timeoutts;
		p_timer.set_interval(std::chrono::seconds(p_current_timeout));
		p_timer.start();
	}
}

std::vector<JausAddress> DiscoveryClient_ReceiveFSM::pGetServices(ReportServices &msg, iop::DiscoveryServiceDef service, unsigned short subsystem)
{
	std::vector<JausAddress> result;
	ReportServices::Body::NodeList *node_list = msg.getBody()->getNodeList();
	RCLCPP_DEBUG(logger, "pGetService search: %s, %d", service.service_uri.c_str(), subsystem);
	for (unsigned int n = 0; n < node_list->getNumberOfElements(); n++) {
		ReportServices::Body::NodeList::NodeSeq *nodes = node_list->getElement(n);
		ReportServices::Body::NodeList::NodeSeq::NodeRec *node = nodes->getNodeRec();
		ReportServices::Body::NodeList::NodeSeq::ComponentList *component_list = nodes->getComponentList();
		for (unsigned int c = 0; c < component_list->getNumberOfElements(); c++) {
			ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq *component_seq = component_list->getElement(c);
			ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq::ComponentRec *component = component_seq->getComponentRec();
			ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq::ServiceList *service_list = component_seq->getServiceList();
			for (unsigned int s = 0; s < service_list->getNumberOfElements(); s++) {
				ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq::ServiceList::ServiceRec *service_rec = service_list->getElement(s);
				RCLCPP_DEBUG(logger, "pGetService: %s", service_rec->getURI().c_str());
				if (service.service_uri.compare(service_rec->getURI()) == 0
						&& service.major_version == service_rec->getMajorVersionNumber()
						&& (service.minor_version == service_rec->getMinorVersionNumber() || service.minor_version == 255)) {
					result.push_back(JausAddress(subsystem, node->getNodeID(), component->getComponentID()));
				}
			}
		}
	}
	return result;
}

std::vector<JausAddress> DiscoveryClient_ReceiveFSM::pGetServices(ReportServiceList &msg, iop::DiscoveryServiceDef service, unsigned short subsystem)
{
	std::vector<JausAddress> result;
	ReportServiceList::Body::SubsystemList *ssys_list = msg.getBody()->getSubsystemList();
	for (unsigned int s = 0; s < ssys_list->getNumberOfElements(); s++) {
		ReportServiceList::Body::SubsystemList::SubsystemSeq *ssystems = ssys_list->getElement(s);
		ReportServiceList::Body::SubsystemList::SubsystemSeq::SubsystemRec *ssystem = ssystems->getSubsystemRec();
		if (ssystem->getSubsystemID() == subsystem) {
			ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList *node_list = ssystems->getNodeList();
			for (unsigned int n = 0; n < node_list->getNumberOfElements(); n++) {
				ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq *nodes = node_list->getElement(n);
				ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::NodeRec *node = nodes->getNodeRec();
				ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList *component_list = nodes->getComponentList();
				for (unsigned int c = 0; c < component_list->getNumberOfElements(); c++) {
					ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentSeq *component_seq = component_list->getElement(c);
					ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentSeq::ComponentRec *component = component_seq->getComponentRec();
					ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentSeq::ServiceList *service_list = component_seq->getServiceList();
					for (unsigned int s = 0; s < service_list->getNumberOfElements(); s++) {
						ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentSeq::ServiceList::ServiceRec *service_rec = service_list->getElement(s);
						if (service.service_uri.compare(service_rec->getURI()) == 0
								&& service.major_version == service_rec->getMajorVersionNumber()
								&& (service.minor_version == service_rec->getMinorVersionNumber() || service.minor_version == 255)) {
							result.push_back(JausAddress(subsystem, node->getNodeID(), component->getComponentID()));
						}
					}
				}
			}
		}
	}
	return result;
}

void DiscoveryClient_ReceiveFSM::appendServiceUri(std::string service_uri, unsigned char major_version, unsigned char minor_version)
{
	iop::DiscoveryServiceDef service;
	service.service_uri = service_uri;
	service.minor_version = minor_version;
	service.major_version = major_version;
	if ( std::find(p_own_uri_services.begin(), p_own_uri_services.end(), service) != p_own_uri_services.end() ) {
		RCLCPP_INFO(logger, "	%s already known", service_uri.c_str());
	} else {
		RCLCPP_DEBUG(logger, "appendServiceUri: %s for registration", service_uri.c_str());
		p_own_uri_services.push_back(service);
		if (register_own_services) {
			p_is_registered = false;
		}
		pCheckTimer();
	}
	for (int i = 0; i < p_discover_services.size(); i++) {
		unsigned short ssid = p_discover_services[i].subsystem;
		JausAddress addr = *(this->jausRouter->getJausAddress());
		if (ssid == 65535) {
			ssid = addr.getSubsystemID();
		}
		if (ssid == addr.getSubsystemID()) {
			if (p_discover_services[i].service.service_uri.compare(service_uri) == 0) {
				// the service was found, forward the address to the callback
				iop::DiscoveryServiceDef service = p_discover_services[i].service;
				RCLCPP_DEBUG(logger, "local service '%s' discovered @%s", service.service_uri.c_str(), addr.str().c_str());
				p_discover_services[i].discovered_in.insert(ssid);
				pInformDiscoverCallbacks(service, addr);
			}
		}
	}
}

void DiscoveryClient_ReceiveFSM::pTimeoutCallback()
{
	this->getHandler()->invoke(p_timeout_event);
	// create a new event, since the InternalEventHandler deletes the given.
	p_timeout_event = new InternalEvent("Timeout", "ControlTimeout");
}

void DiscoveryClient_ReceiveFSM::registerAction()
{
	/// Insert User Code HERE
	if (p_first_ready) {
		sendQueryIdentificationAction();
		p_first_ready = false;
	}
}

void DiscoveryClient_ReceiveFSM::handleReportConfigurationAction(ReportConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	RCLCPP_DEBUG(logger, "reportedConfigurationAction from %s, not implemented", transportData.getAddress().str().c_str());
}

void DiscoveryClient_ReceiveFSM::handleReportIdentificationAction(ReportIdentification msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	JausAddress sender = transportData.getAddress();
	bool request_query_srvs = p_ros_interface.update_ident(sender, msg);

	unsigned char request_type = msg.getBody()->getReportIdentificationRec()->getQueryType();
	unsigned char system_type = msg.getBody()->getReportIdentificationRec()->getType();
	std::string name = msg.getBody()->getReportIdentificationRec()->getIdentification();

	RCLCPP_DEBUG(logger, "reportedIdentificationAction from sender: %s, request_type: %d, system_type: %d, name: %s",
			sender.str().c_str(), (int)request_type, (int)system_type, name.c_str());
	unsigned short subsystem_id = sender.getSubsystemID();
	if (request_type == TYPE_SUBSYSTEM) {
		uint64_t now_sec = iop::now_secs();
		ServiceRequests& srv_req = p_get_service_request(sender);
		bool doupdate = request_query_srvs;
		bool has_to_discover = pHasToDiscover(subsystem_id) && srv_req.allow_send(now_sec);
		if (has_to_discover || doupdate || p_on_registration) {
			// request nodes of the subsystem
			query_identification(TYPE_NODE, subsystem_id, 0xFF, 0xFF);
			if (has_to_discover) {
				RCLCPP_DEBUG(logger, "  request services while discover for requested services");
			}
			if (request_query_srvs) {
				RCLCPP_DEBUG(logger, "  request services to update ROS interface");
			}
			if (p_on_registration) {
				RCLCPP_DEBUG(logger, "  request services to check own registered services");
			}
			// request configuration
			QueryConfiguration req_cfg_msg;
			req_cfg_msg.getBody()->getQueryConfigurationRec()->setQueryType(request_type);
			this->sendJausMessage(req_cfg_msg, sender);
			if (!srv_req.received_list and srv_req.count > 3) {
				RCLCPP_DEBUG(logger, "send request for QueryServices for compatibility to v1.0  to %d.%d.%d", sender.getSubsystemID(), sender.getNodeID(), sender.getComponentID());
				QueryServices req_srv_msg;
				this->sendJausMessage(req_srv_msg, sender);
			}
			RCLCPP_DEBUG(logger, "send request for QueryServiceList to %d.%d.%d", sender.getSubsystemID(), sender.getNodeID(), sender.getComponentID());
			QueryServiceList req_srvl_msg;
			QueryServiceList::Body::SubsystemList *sslist = req_srvl_msg.getBody()->getSubsystemList();
			QueryServiceList::Body::SubsystemList::SubsystemSeq ssr;
			ssr.getSubsystemRec()->setSubsystemID(65535);
			sslist->addElement(ssr);
			QueryServiceList::Body::SubsystemList::SubsystemSeq *ssrec = sslist->getElement(sslist->getNumberOfElements()-1);
			QueryServiceList::Body::SubsystemList::SubsystemSeq::NodeList *nlist = ssrec->getNodeList();
			QueryServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq nr;
			nr.getNodeRec()->setNodeID(255);
			nlist->addElement(nr);
			QueryServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq *nrseq = nlist->getElement(nlist->getNumberOfElements()-1);
			QueryServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList *clist = nrseq->getComponentList();
			QueryServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentRec cr;
			cr.setComponentID(255);
			clist->addElement(cr);
			this->sendJausMessage(req_srvl_msg, sender);
			srv_req.count += 1;
			srv_req.ts_last_request = now_sec;
		}
		p_discovery_srvs_stamps[sender.getSubsystemID()] = iop::now_secs();
	}
}

void DiscoveryClient_ReceiveFSM::handleReportServiceListAction(ReportServiceList msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ServiceRequests& srv_req = p_get_service_request(sender);
	srv_req.received_list = true;
	srv_req.count = 0;
	RCLCPP_DEBUG(logger, "handleReportServiceListAction from sender: %s", sender.str().c_str());
	p_discovery_srvs_stamps[sender.getSubsystemID()] = iop::now_secs();
	// This message is received after own services are registered -> test is service registered?
	// or after QueryServices was send
	if (jausRouter->getJausAddress()->getSubsystemID() == sender.getSubsystemID()) {
		// -> test is service registered?
		std::set<std::string> services_registered;
		// create a list with all service URI's associated with own JAUS address
		ReportServiceList::Body::SubsystemList *ssys_list = msg.getBody()->getSubsystemList();
		for (unsigned int s = 0; s < ssys_list->getNumberOfElements(); s++) {
			ReportServiceList::Body::SubsystemList::SubsystemSeq *ssystems = ssys_list->getElement(s);
			ReportServiceList::Body::SubsystemList::SubsystemSeq::SubsystemRec *ssystem = ssystems->getSubsystemRec();
			if (ssystem->getSubsystemID() == jausRouter->getJausAddress()->getSubsystemID()) {
				ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList *node_list = ssystems->getNodeList();
				for (unsigned int n = 0; n < node_list->getNumberOfElements(); n++) {
					ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq *nodes = node_list->getElement(n);
					ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::NodeRec *node = nodes->getNodeRec();
					if (node->getNodeID() == jausRouter->getJausAddress()->getNodeID()) {
						ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList *component_list = nodes->getComponentList();
						for (unsigned int c = 0; c < component_list->getNumberOfElements(); c++) {
							ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentSeq *component_seq = component_list->getElement(c);
							ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentSeq::ComponentRec *component = component_seq->getComponentRec();
							if (component->getComponentID() == jausRouter->getJausAddress()->getComponentID()) {
								ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentSeq::ServiceList *service_list = component_seq->getServiceList();
								for (unsigned int s = 0; s < service_list->getNumberOfElements(); s++) {
									ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentSeq::ServiceList::ServiceRec *service = service_list->getElement(s);
									services_registered.insert(service->getURI());
								}
							}
						}
					}
				}
			}
		}
		p_is_registered = true;
		if (register_own_services) {
			// test all own URI's are in the registered list
			for (unsigned int i = 0; i < p_own_uri_services.size(); i++) {
				if (std::find(services_registered.begin(), services_registered.end(), p_own_uri_services[i].service_uri) == services_registered.end()) {
					RCLCPP_WARN(logger, "own service '%s' not found in reported services! retry registration...",
							p_own_uri_services[i].service_uri.c_str());
					p_is_registered = false;
					p_on_registration = false;
				}
			}
			if (p_is_registered && p_on_registration) {
				RCLCPP_INFO(logger, "all services are registered!");
			}
		}
		if (p_is_registered && p_on_registration) {
			p_on_registration = false;
			pRegistrationFinished();
		}
	}
	lock_type lock(p_mutex);
	// update ROS interface
	p_ros_interface.update_services(sender, msg);
	// check for services to discover, but only if a discovery_handler is set
	for (int i = p_discover_services.size()-1; i >= 0; --i) {
		if (!p_discover_services[i].discovered(transportData.getSourceID()->getSubsystemID())) {
			RCLCPP_DEBUG(logger, "  discover %s, subsystem: %d", p_discover_services[i].service.service_uri.c_str(), p_discover_services[i].subsystem);
		}
	}
	for (int ds_idx = 0; ds_idx < p_discover_services.size(); ds_idx++) {
//		if (!p_discover_services[i].discovered) {
			unsigned short ssid = p_discover_services[ds_idx].subsystem;
			if (ssid == 65535) {
				ssid = transportData.getSourceID()->getSubsystemID();
			}
			if (ssid == transportData.getSourceID()->getSubsystemID()) {
				std::vector<JausAddress> srvs = this->pGetServices(msg, p_discover_services[ds_idx].service, ssid);
				for (unsigned int addr_idx = 0; addr_idx < srvs.size(); addr_idx++) {
					JausAddress addr = srvs[addr_idx];
					// the service was found, forward the address to the callback
					iop::DiscoveryServiceDef service = p_discover_services[ds_idx].service;
					RCLCPP_DEBUG(logger, "service '%s' discovered @%d.%d.%d through service list", service.service_uri.c_str(), addr.getSubsystemID(), addr.getNodeID(), addr.getComponentID());
					p_discover_services[ds_idx].discovered_in.insert(transportData.getSourceID()->getSubsystemID());
					pInformDiscoverCallbacks(service, addr);
				}
				if (srvs.size() == 0) {
					p_discover_services[ds_idx].discovered_in.erase(transportData.getSourceID()->getSubsystemID());
				}
			}
//		}
	}
	pCheckTimer();
}

void DiscoveryClient_ReceiveFSM::handleReportServicesAction(ReportServices msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ServiceRequests& srv_req = p_get_service_request(sender);
	srv_req.count = 0;
	RCLCPP_DEBUG(logger, "handleReportServicesAction from sender: %s", sender.str().c_str());
	/// Insert User Code HERE
	// This message is received after own services are registered -> test is service registered?
	// or after QueryServices was send
	if (jausRouter->getJausAddress()->getSubsystemID() == sender.getSubsystemID()) {
		// -> test is service registered?
		std::set<std::string> services_registered;
		// create a list with all service URI's associated with own JAUS address
		ReportServices::Body::NodeList *node_list = msg.getBody()->getNodeList();
		for (unsigned int n = 0; n < node_list->getNumberOfElements(); n++) {
			ReportServices::Body::NodeList::NodeSeq *nodes = node_list->getElement(n);
			ReportServices::Body::NodeList::NodeSeq::NodeRec *node = nodes->getNodeRec();
			if (node->getNodeID() == jausRouter->getJausAddress()->getNodeID()) {
				ReportServices::Body::NodeList::NodeSeq::ComponentList *component_list = nodes->getComponentList();
				for (unsigned int c = 0; c < component_list->getNumberOfElements(); c++) {
					ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq *component_seq = component_list->getElement(c);
					ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq::ComponentRec *component = component_seq->getComponentRec();
					if (component->getComponentID() == jausRouter->getJausAddress()->getComponentID()) {
						ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq::ServiceList *service_list = component_seq->getServiceList();
						for (unsigned int s = 0; s < service_list->getNumberOfElements(); s++) {
							ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq::ServiceList::ServiceRec *service = service_list->getElement(s);
							services_registered.insert(service->getURI());
						}
					}
				}
			}
		}
		p_is_registered = true;
		if (register_own_services) {
			// test all own URI's are in the registered list
			for (unsigned int i = 0; i < p_own_uri_services.size(); i++) {
				if (std::find(services_registered.begin(), services_registered.end(), p_own_uri_services[i].service_uri) == services_registered.end()) {
					RCLCPP_WARN(logger, "own service '%s' not found in reported services! retry registration...",
							p_own_uri_services[i].service_uri.c_str());
					p_is_registered = false;
					p_on_registration = false;
				}
			}
			if (p_is_registered && p_on_registration) {
				RCLCPP_INFO(logger, "all services are registered! (ReportServices checked)");
			}
		}
		if (p_is_registered && p_on_registration) {
			p_on_registration = false;
			pRegistrationFinished();
		}
	}
	lock_type lock(p_mutex);
	// update ROS interface
	p_ros_interface.update_services(sender, msg);
	// check for services to discover, but only if a discovery_handler is set
	for (int i = p_discover_services.size()-1; i >= 0; --i) {
		if (!p_discover_services[i].discovered(transportData.getSourceID()->getSubsystemID())) {
			RCLCPP_DEBUG(logger, "  discover %s, subsystem: %d", p_discover_services[i].service.service_uri.c_str(), p_discover_services[i].subsystem);
		}
	}
	for (int ds_idx = 0; ds_idx < p_discover_services.size(); ds_idx++) {
//		if (!p_discover_services[i].discovered) {
			unsigned short ssid = p_discover_services[ds_idx].subsystem;
			if (ssid == 65535) {
				ssid = transportData.getSourceID()->getSubsystemID();
			}
			if (ssid == transportData.getSourceID()->getSubsystemID()) {
				std::vector<JausAddress> srvs = this->pGetServices(msg, p_discover_services[ds_idx].service, ssid);
				for (unsigned int addr_idx = 0; addr_idx < srvs.size(); addr_idx++) {
					JausAddress addr = srvs[addr_idx];
					// the service was found, forward the address to the callback
					iop::DiscoveryServiceDef service = p_discover_services[ds_idx].service;
					RCLCPP_DEBUG(logger, "service '%s' discovered @%d.%d.%d through services old stile, using QueryServices", service.service_uri.c_str(), addr.getSubsystemID(), addr.getNodeID(), addr.getComponentID());
					p_discover_services[ds_idx].discovered_in.insert(transportData.getSourceID()->getSubsystemID());
					pInformDiscoverCallbacks(service, addr);
				}
				if (srvs.size() == 0) {
					p_discover_services[ds_idx].discovered_in.erase(transportData.getSourceID()->getSubsystemID());
				}
			}
//		}
	}
	pCheckTimer();
}

void DiscoveryClient_ReceiveFSM::handleReportSubsystemListAction(ReportSubsystemList msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	RCLCPP_DEBUG(logger, "reportedSubsystemList from sender: subsystem: %d, node: %d,	component: %d",
			transportData.getSourceID()->getSubsystemID(), transportData.getSourceID()->getNodeID(), transportData.getSourceID()->getComponentID());
	ReportSubsystemList::Body::SubsystemList *dis_subsystems = msg.getBody()->getSubsystemList();
	RCLCPP_DEBUG(logger, "  known subsystems: %u", dis_subsystems->getNumberOfElements());
	for (int i=0; i < dis_subsystems->getNumberOfElements(); i++) {
		ReportSubsystemList::Body::SubsystemList::SubsystemRec *item = dis_subsystems->getElement(i);
		RCLCPP_DEBUG(logger, "  subsystem: %d, node: %d, component: %d",
				item->getSubsystemID(),item->getNodeID(), item->getComponentID());
	}
}

void DiscoveryClient_ReceiveFSM::sendQueryIdentificationAction()
{
	// remove expired subsystems (not responding to query identification messages)
	p_check_for_timeout_discovery_service();
	for (unsigned int jidx = 0; jidx < p_unicast_subsystems.size(); jidx++) {
		// send discovery to unicast addresses
		JausAddress addr = p_unicast_subsystems[jidx];
		query_identification(TYPE_SUBSYSTEM, addr.getSubsystemID(), addr.getNodeID(), addr.getComponentID());
	}
	if (p_ros_interface.enabled()) {
		// if ROS interface enabled we send component queries to get names for components
		query_identification(TYPE_COMPONENT, 0xFFFF, 0xFF, 0xFF);
	}
	if (!(pHasToDiscover(65535) || p_ros_interface.enabled())) {
		// if we descovered all services and have enabled ROS interface we send subsystem and component queries
		query_identification(TYPE_SUBSYSTEM, 0xFFFF, 0xFF, 0xFF);
		return;
	}
	int query_type = TYPE_SUBSYSTEM;
	unsigned short subsystem_id = jausRouter->getJausAddress()->getSubsystemID();
	if (system_id == TYPE_SUBSYSTEM) {
		// we are subsystem -> discover system
		query_type = TYPE_SYSTEM;
		subsystem_id = 65535; //0xFFFF
	} else if (system_id == TYPE_NODE or
		// we are node -> find subsystem discovery service and register own services
		// if this discovery service is included in a component.
			system_id == TYPE_COMPONENT) {
		// IOP 5a: A QueryIdentification message shall be broadcast by
		// every JAUS component on the subsystem at a rate of at least once per 5 minutes for the
		// purpose of finding and registering services with the Discovery service.

		// is set by default...
	}
	query_identification(TYPE_SUBSYSTEM, 0xFFFF, 0xFF, 0xFF);
	if (query_type != TYPE_SUBSYSTEM) {
		query_identification(query_type, subsystem_id, 0xFF, 0xFF);
		// send query for services to discover, if these are not in the same subsystem
		if (p_discover_services.size() > 0) {
			std::set<int> subsystems;
			subsystems.insert(subsystem_id);
			lock_type lock(p_mutex);
			for (unsigned int i = 0; i < p_discover_services.size(); i++) {
				if (!p_discover_services[i].discovered()) {
					subsystem_id = p_discover_services[i].subsystem;
					if (subsystems.count(subsystem_id) == 0) {
						subsystems.insert(subsystem_id);
						RCLCPP_DEBUG(logger, "send QueryServices to subsystem: %d for discover service: %s",
								subsystem_id, p_discover_services[i].service.service_uri.c_str());
						query_identification(query_type, subsystem_id, 0xFF, 0xFF);
		//				QueryIdentification msg;
		//				msg.getBody()->getQueryIdentificationRec()->setQueryType(TYPE_SUBSYSTEM);
		//				printf("[DiscoveryClient] send QueryIdentification to subsystem: %d of type: %d (SUBSYSTEM)\n", subsystem_id, TYPE_SUBSYSTEM);
		//				sendJausMessage(msg,JausAddress(subsystem_id, 0xFF, 0xFF)); //0xFFFF, 0xFF, 0xFF
					}
				}
			}
		}
	}
	if (p_count_discover_tries <= TIMEOUT_STANDBY / TIMEOUT_DISCOVER) {
		p_count_discover_tries++;
	}

	p_timer.start();
}

void DiscoveryClient_ReceiveFSM::sendRegisterServicesAction(ReportIdentification msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	int query_type = msg.getBody()->getReportIdentificationRec()->getQueryType();
	RCLCPP_DEBUG(logger, "sendRegisterServices query_type: %d, sender: subsystem: %d, node: %d,	component: %d",
			query_type, transportData.getSourceID()->getSubsystemID(), transportData.getSourceID()->getNodeID(), transportData.getSourceID()->getComponentID());
	if (p_own_uri_services.size() == 0) {
		// do not register, if we have no services to register setted by appendServiceUri()
		handleReportIdentificationAction(msg, transportData);
		return;
	}
	if (query_type == TYPE_SUBSYSTEM) {
		if (this->jausRouter->getJausAddress()->getSubsystemID() == transportData.getSourceID()->getSubsystemID()) {
			if (!onRegistration() && !isRegistered()) {
				p_on_registration = true;
				p_addr_discovery_service = transportData.getAddress();
				RCLCPP_INFO(logger, "register services by [%s]", p_addr_discovery_service.str().c_str());
				RCLCPP_DEBUG(logger, "Discovery service info:");
				RCLCPP_DEBUG(logger, "  QueryType: %d", msg.getBody()->getReportIdentificationRec()->getQueryType());
				RCLCPP_DEBUG(logger, "  Type     : %d", msg.getBody()->getReportIdentificationRec()->getType());
				RCLCPP_DEBUG(logger, "  ID       : %s", msg.getBody()->getReportIdentificationRec()->getIdentification().c_str());
				RegisterServices msg_reg_services;
				RegisterServices::RegisterServicesBody::ServiceList list_services;
				for (unsigned int i = 0; i < p_own_uri_services.size(); i++) {
					iop::DiscoveryServiceDef &service = p_own_uri_services[i];
					RegisterServices::RegisterServicesBody::ServiceList::ServiceRec service_rec;
					service_rec.setMinorVersionNumber(service.minor_version);
					service_rec.setMajorVersionNumber(service.major_version);
					service_rec.setURI(service.service_uri);
					list_services.addElement(service_rec);
				}
				msg_reg_services.getRegisterServicesBody()->setServiceList(list_services);
				sendJausMessage(msg_reg_services, p_addr_discovery_service);

				RCLCPP_DEBUG(logger, "send QueryServices to validate the registration");
				JausAddress ownaddr = *this->jausRouter->getJausAddress();
				QueryServiceList req_srvl_msg;
				QueryServiceList::Body::SubsystemList *sslist = req_srvl_msg.getBody()->getSubsystemList();
				QueryServiceList::Body::SubsystemList::SubsystemSeq ssr;
				ssr.getSubsystemRec()->setSubsystemID(ownaddr.getSubsystemID());
				sslist->addElement(ssr);
				QueryServiceList::Body::SubsystemList::SubsystemSeq *ssrec = sslist->getElement(sslist->getNumberOfElements()-1);
				QueryServiceList::Body::SubsystemList::SubsystemSeq::NodeList *nlist = ssrec->getNodeList();
				QueryServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq nr;
				nr.getNodeRec()->setNodeID(ownaddr.getNodeID());
				nlist->addElement(nr);
				QueryServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq *nrseq = nlist->getElement(nlist->getNumberOfElements()-1);
				QueryServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList *clist = nrseq->getComponentList();
				QueryServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentRec cr;
				cr.setComponentID(ownaddr.getComponentID());
				clist->addElement(cr);
//				QueryServices msg;
//				QueryServices::Body::NodeList nlist;
//				QueryServices::Body::NodeList::NodeSeq nseq;
//				QueryServices::Body::NodeList::NodeSeq::NodeRec nreq;
//				QueryServices::Body::NodeList::NodeSeq::ComponentList clist;
//				QueryServices::Body::NodeList::NodeSeq::ComponentList::ComponentRec creq;
//
//				nreq.setNodeID(jausRouter->getJausAddress()->getNodeID());
//				creq.setComponentID(jausRouter->getJausAddress()->getComponentID());
//				clist.addElement(creq);
//				nseq.setComponentList(clist);
//				nseq.setNodeRec(nreq);
//				nlist.addElement(nseq);
//				msg.getBody()->setNodeList(nlist);
				sendJausMessage(req_srvl_msg, p_addr_discovery_service);
			}
		} else {
			RCLCPP_DEBUG(logger, "  skip registration on this discovery service because of different subsystem!");
		}
	} else if (query_type == TYPE_SYSTEM) {
		// add a discovery service for ReportSubsystemLIst
		if (p_discovery_fsm != NULL) {
			p_discovery_fsm->registerSubsystem(transportData.getAddress());
		}
	}
}

void DiscoveryClient_ReceiveFSM::p_check_for_timeout_discovery_service()
{
	if (p_timeout_discover_service > 0) {
		std::vector<unsigned short> to_remove;
		std::map<unsigned short, unsigned int>::iterator it;
		int64_t now = iop::now_secs();
		for (it = p_discovery_srvs_stamps.begin(); it != p_discovery_srvs_stamps.end(); it++) {
			if (it->second > 0) {
				if (now - it->second > p_timeout_discover_service) {
					// it was the discovery service where we register our services. We need to register again.
					if (it->first == p_addr_discovery_service.getSubsystemID()) {
						if (p_is_registered) {
							RCLCPP_WARN(logger, "discovery service timeout, set service to not registered!");
						}
						p_is_registered = false;
					}
					to_remove.push_back(it->first);
				}
			}
		}
		// remove expired subsystems
		for (unsigned int i = 0; i < to_remove.size(); i++) {
			p_discovery_srvs_stamps.erase(p_discovery_srvs_stamps.find(to_remove[i]));
		}
	}
}

bool DiscoveryClient_ReceiveFSM::isRegistered()
{
	p_check_for_timeout_discovery_service();
	return p_is_registered || (p_own_uri_services.size() == 0);
}

bool DiscoveryClient_ReceiveFSM::onRegistration()
{
	/// Insert User Code HERE
	return p_on_registration;
}

void DiscoveryClient_ReceiveFSM::pDiscover(std::string service_uri, unsigned char major_version, unsigned char minor_version, unsigned short subsystem)
{
	p_count_discover_tries = 0;
	iop::DiscoveryServiceDef service(service_uri, major_version, minor_version);
	JausAddress own_addr = *(this->jausRouter->getJausAddress());
	bool own_found = false;
	lock_type lock(p_mutex);
	for (unsigned int i = 0; i < p_discover_services.size(); i++) {
		if (p_discover_services[i].service == service
				&& p_discover_services[i].subsystem == subsystem) {
			RCLCPP_DEBUG(logger, "%s %d.%d for subsystem: %d already in discover, skip", service_uri.c_str(), major_version, minor_version, subsystem);
			p_discover_services[i].discovered_in.clear();
			return;
		}
	}
	RCLCPP_DEBUG(logger, "added %s %d.%d for subsystem: %d to discovery", service_uri.c_str(), major_version, minor_version, subsystem);
	DiscoverItem di;
	di.service = service;
	di.subsystem = subsystem;
	// search in own services
	unsigned short ssid = subsystem;
	if (ssid == 65535) {
		ssid = own_addr.getSubsystemID();
	}
	if (ssid == own_addr.getSubsystemID()) {
		if ( std::find(p_own_uri_services.begin(), p_own_uri_services.end(), service) != p_own_uri_services.end() ) {
			RCLCPP_DEBUG(logger, "	discovered own service %s", service_uri.c_str());
			di.discovered_in.insert(ssid);
			own_found = true;
		}
	}
	p_discover_services.push_back(di);
	if (own_found) {
		pInformDiscoverCallbacks(service, own_addr);
	}
//	int query_type = TYPE_SUBSYSTEM;
//	unsigned short subsystem_id = subsystem;
//	QueryIdentification msg;
//	msg.getBody()->getQueryIdentificationRec()->setQueryType(query_type);
//	RCLCPP_DEBUG(logger, "discover, send QueryIdentification to subsystem: %d", subsystem_id);
//	sendJausMessage(msg,JausAddress(subsystem_id, 0xFF, 0xFF)); //0xFFFF, 0xFF, 0xFF
	pCheckTimer();
}

void DiscoveryClient_ReceiveFSM::cancel_discovery()
{
	lock_type lock(p_mutex);
	p_discover_services.clear();
}

void DiscoveryClient_ReceiveFSM::cancel_discovery(std::string service_uri, unsigned char major_version, unsigned char minor_version)
{
	lock_type lock(p_mutex);
	iop::DiscoveryServiceDef service(service_uri, major_version, minor_version);
	for (unsigned int i = 0; i < p_discover_services.size(); i++) {
		if (p_discover_services[i].service == service) {
			RCLCPP_DEBUG(logger, "remove %s from discover", service_uri.c_str());
			p_discover_services.erase(p_discover_services.begin() + i);
			return;
		}
	}
}

void DiscoveryClient_ReceiveFSM::query_identification(int query_type, jUnsignedShortInteger subsystem, jUnsignedByte node, jUnsignedByte component)
{
	QueryIdentification msg;
	msg.getBody()->getQueryIdentificationRec()->setQueryType(query_type);
	RCLCPP_DEBUG(logger, "send QueryIdentification to subsystem.node.comp: %i.%d.%d of type %d, next query in %i sec",
			subsystem, (int)node, (int)component, query_type, p_current_timeout);
	if ((p_current_diagnostic_level == 0 || p_current_diagnostic_level == 3) && (!p_is_registered || pHasToDiscover(65535))) {
		dynamic_cast<iop::IopJausRouter*>(jausRouter)->getComponent()->send_diagnostic(3, "Discover services");
	}
	sendJausMessage(msg,JausAddress(subsystem, node, component)); //0xFFFF, 0xFF, 0xFF
}

bool DiscoveryClient_ReceiveFSM::pHasToDiscover(unsigned short subsystem_id)
{
	for (int i = p_discover_services.size()-1; i >= 0; --i) {
		if (subsystem_id == 65535) {
			// go through all discovered robots
			std::map<unsigned short, unsigned int>::iterator it;
			for (it = p_discovery_srvs_stamps.begin(); it != p_discovery_srvs_stamps.end(); ++it) {
				if (!p_discover_services[i].discovered(it->first)) {
					return true;
				}
			}
		} else {
			if (!p_discover_services[i].discovered(subsystem_id)) {
				return true;
			}
		}
	}
	return false;
}

void DiscoveryClient_ReceiveFSM::pInformDiscoverCallbacks(iop::DiscoveryServiceDef &service, JausAddress &address)
{
	std::map <iop::DiscoveryServiceDef, std::vector<std::function<void (const std::string &, JausAddress &)> > > ::iterator it;
	it = p_discover_callbacks.find(service);
	if (it != p_discover_callbacks.end()) {
		for (unsigned int i = 0; i < it->second.size(); i++) {
			it->second[i](service.service_uri, address);
		}
//		it->second.clear();
//		p_discover_callbacks.erase(it);
	}
	if (class_discovery_callback_ != nullptr) {
		class_discovery_callback_(service.service_uri, address);
	}

}

DiscoveryClient_ReceiveFSM::ServiceRequests& DiscoveryClient_ReceiveFSM::p_get_service_request(JausAddress &discovery_service)
{
	std::map<JausAddress, ServiceRequests>::iterator it = p_service_requests.find(discovery_service);
	if (it == p_service_requests.end()) {
		DiscoveryClient_ReceiveFSM::ServiceRequests req;
		req.count = 0;
		req.received_list = false;
		req.ts_last_request = 0;
		p_service_requests[discovery_service] = req;
	}
	return p_service_requests[discovery_service];
}

}
