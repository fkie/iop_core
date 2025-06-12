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

#include <fkie_iop_discovery/DiscoveryRosInterface.h>
#include <algorithm>
#include <fkie_iop_component/iop_config.hpp>

using namespace iop;
using namespace urn_jaus_jss_core_DiscoveryClient;

DiscoveryRosInterface::DiscoveryRosInterface(std::shared_ptr<iop::Component> cmp)
: logger(cmp->get_logger().get_child("DiscoveryClient")),
  p_components(logger)
{
	p_jaus_router = NULL;
	p_enable_ros_interface = false;
	p_force_component_update_after = 300;
	p_timeout_discover_service = 60;
}

void DiscoveryRosInterface::setup(std::shared_ptr<iop::Component> cmp, JTS::StateMachine& jaus_router)
{
	p_jaus_router = &jaus_router;
	iop::Config cfg(cmp, "DiscoveryClient");
	cfg.declare_param<bool>("enable_ros_interface", p_enable_ros_interface, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
		"Enables ROS interface for using with ROS operator control unit (e.g. RQt or RViz)",
		"Default: false");
	cfg.declare_param<int64_t>("force_component_update_after", p_force_component_update_after, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
		"Discovery client updates all discovered services after given time",
		"Default: 300 sec");
	cfg.param("enable_ros_interface", p_enable_ros_interface, p_enable_ros_interface);
	cfg.param("force_component_update_after", p_force_component_update_after, p_force_component_update_after);
	if (p_enable_ros_interface) {
		p_pub_identification = cfg.create_publisher<fkie_iop_msgs::msg::Identification>("/iop_identification", 10);
		rclcpp::QoS qos_latched(10);
		p_pub_system = cfg.create_publisher<fkie_iop_msgs::msg::System>("/iop_system", qos_latched.transient_local());
		p_srv_query_ident = cfg.create_service<fkie_iop_msgs::srv::QueryIdentification>("/iop_query_identification", std::bind(&DiscoveryRosInterface::pQueryIdentificationSrv, this, std::placeholders::_1, std::placeholders::_2));
		p_srv_update_system = cfg.create_service<std_srvs::srv::Empty>("/iop_update_discovery", std::bind(&DiscoveryRosInterface::pUpdateSystemSrv, this, std::placeholders::_1, std::placeholders::_2));
	}
}

void DiscoveryRosInterface::set_discovery_timeout(int64_t timeout)
{
	p_timeout_discover_service = timeout;
	p_components.set_timeout(p_timeout_discover_service);
}

void DiscoveryRosInterface::pQueryIdentificationSrv(const std::shared_ptr<fkie_iop_msgs::srv::QueryIdentification::Request> req, std::shared_ptr<fkie_iop_msgs::srv::QueryIdentification::Response> /* res */)
{
	p_query_identification(req->type, 0xFFFF, 0xFF, 0xFF);
}

void DiscoveryRosInterface::pUpdateSystemSrv(const std::shared_ptr<std_srvs::srv::Empty::Request> /* req */, std::shared_ptr<std_srvs::srv::Empty::Response> /* res */)
{
	p_discovery_srvs_stamps.clear();
	p_query_identification(2, 0xFFFF, 0xFF, 0xFF);
}

void DiscoveryRosInterface::p_query_identification(int query_type, jUnsignedShortInteger subsystem, jUnsignedByte node, jUnsignedByte component)
{
	JausAddress dest(subsystem, node, component);
	urn_jaus_jss_core_DiscoveryClient::QueryIdentification msg;
	msg.getBody()->getQueryIdentificationRec()->setQueryType(query_type);
	RCLCPP_DEBUG(logger, "send QueryIdentification by ROS request to subsystem.node.comp: %s of type %d", dest.str().c_str(), query_type);
	p_jaus_router->sendJausMessage(msg, dest); //0xFFFF, 0xFF, 0xFF
}

bool DiscoveryRosInterface::update_ident(JausAddress &addr, ReportIdentification &report_ident)
{
	if (!p_enable_ros_interface) {
		return false;
	}
	bool result = false;
	auto ident = fkie_iop_msgs::msg::Identification();
	ident.name = report_ident.getBody()->getReportIdentificationRec()->getIdentification();
	ident.request_type = report_ident.getBody()->getReportIdentificationRec()->getQueryType();
	ident.system_type = report_ident.getBody()->getReportIdentificationRec()->getType();
	ident.address = p_convert(addr);

	switch (report_ident.getBody()->getReportIdentificationRec()->getQueryType()) {
		case 1: {  // SYSTEM TYPE
			break;
		}
		case 2: {  // SUBSYSTEM TYPE
			p_subsystem_idents[addr] = ident;
			p_components.update_ts(addr, addr.getSubsystemID());
			std::map<JausAddress, int64_t>::iterator itts = p_discovery_srvs_stamps.find(addr);
			if (itts == p_discovery_srvs_stamps.end()) {
				result = true;
			} else if (p_force_component_update_after > 0) {
				if (iop::Component::now_secs() - itts->second > p_force_component_update_after) {
					result = true;
				}
			}
			break;
		}
		case 3: {  // NODE TYPE
			p_node_idents[p_get_node_addr(addr)] = ident;
			p_components.update_ts(addr, addr.getSubsystemID(), addr.getNodeID());
			break;
		}
		case 4: {  // COMPONENT TYPE
			break;
		}
	}
	p_pub_identification->publish(ident);
	return result;
}

void DiscoveryRosInterface::update_services(JausAddress discovery_addr, ReportServiceList msg)
{
	if (!p_enable_ros_interface) {
		return;
	}
	// Create ROS message from components
	p_discovery_srvs_stamps[discovery_addr] = iop::Component::now_secs();
	p_components.remove_discovery_service(discovery_addr);
	// update the object with discovered system and publish it to ROS
	ReportServiceList::Body::SubsystemList *ssys_list = msg.getBody()->getSubsystemList();
	for (unsigned int s = 0; s < ssys_list->getNumberOfElements(); s++) {
		ReportServiceList::Body::SubsystemList::SubsystemSeq *ssystems = ssys_list->getElement(s);
		ReportServiceList::Body::SubsystemList::SubsystemSeq::SubsystemRec *ssystem = ssystems->getSubsystemRec();
		ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList *node_list = ssystems->getNodeList();
		for (unsigned int n = 0; n < node_list->getNumberOfElements(); n++) {
			ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq *nodes = node_list->getElement(n);
			ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::NodeRec *node = nodes->getNodeRec();
			ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList *component_list = nodes->getComponentList();
			for (unsigned int c = 0; c < component_list->getNumberOfElements(); c++) {
				ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentSeq *component_seq = component_list->getElement(c);
				ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentSeq::ComponentRec *component = component_seq->getComponentRec();
				JausAddress cmpaddr(ssystem->getSubsystemID(), node->getNodeID(), component->getComponentID());
				for (unsigned int si = 0; si < component_seq->getServiceList()->getNumberOfElements(); si++) {
					ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentSeq::ServiceList::ServiceRec *service = component_seq->getServiceList()->getElement(si);
					p_components.add_service(discovery_addr, cmpaddr, service->getURI(), service->getMajorVersionNumber(), service->getMinorVersionNumber());
				}
			}
		}
	}
	p_publish_subsystem();
}

void DiscoveryRosInterface::update_services(JausAddress discovery_addr, ReportServices msg)
{
	if (!p_enable_ros_interface) {
		return;
	}
	// Create ROS message from components
	p_discovery_srvs_stamps[discovery_addr] = iop::Component::now_secs();
	p_components.remove_discovery_service(discovery_addr);
	// update the object with discovered system and publish it to ROS
	ReportServices::Body::NodeList *node_list = msg.getBody()->getNodeList();
	for (unsigned int n = 0; n < node_list->getNumberOfElements(); n++) {
		ReportServices::Body::NodeList::NodeSeq *nodes = node_list->getElement(n);
		ReportServices::Body::NodeList::NodeSeq::NodeRec *node = nodes->getNodeRec();
		ReportServices::Body::NodeList::NodeSeq::ComponentList *component_list = nodes->getComponentList();
		for (unsigned int c = 0; c < component_list->getNumberOfElements(); c++) {
			ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq *component_seq = component_list->getElement(c);
			ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq::ComponentRec *component = component_seq->getComponentRec();
			JausAddress cmpaddr(discovery_addr.getSubsystemID(), node->getNodeID(), component->getComponentID());
			for (unsigned int si = 0; si < component_seq->getServiceList()->getNumberOfElements(); si++) {
				ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq::ServiceList::ServiceRec *service = component_seq->getServiceList()->getElement(si);
				p_components.add_service(discovery_addr, cmpaddr, service->getURI(), service->getMajorVersionNumber(), service->getMinorVersionNumber());
			}
		}
	}
	p_publish_subsystem();
}

bool DiscoveryRosInterface::enabled()
{
	return p_enable_ros_interface;
}

void DiscoveryRosInterface::p_publish_subsystem()
{
	auto system = fkie_iop_msgs::msg::System();
	std::vector<JausAddress> discover_services = p_components.get_discovery_services();
	std::vector<JausAddress>::iterator itds;
	for (itds = discover_services.begin(); itds != discover_services.end(); ++itds) {
		// create subsystem
		fkie_iop_msgs::msg::Subsystem ss_new;
		std::map<JausAddress, fkie_iop_msgs::msg::Identification>::iterator itidss = p_subsystem_idents.find(*itds);
		if (itidss != p_subsystem_idents.end()) {
			ss_new.ident = itidss->second;
		}
		std::map<JausAddress, fkie_iop_msgs::msg::Subsystem>::iterator it;
		std::vector<DiscoveryComponent> components = p_components.get_components(*itds);
		std::vector<DiscoveryComponent>::iterator itcmp;
		for (itcmp = components.begin(); itcmp != components.end(); ++itcmp) {
			// create a new node if need. this node is already added to subsystem
			fkie_iop_msgs::msg::Node& ros_node = p_get_node(ss_new, itcmp->address);
			// create component
			fkie_iop_msgs::msg::Component ros_cmpt;
			ros_cmpt.address = p_convert(itcmp->address);
			std::vector<iop::DiscoveryServiceDef> services = itcmp->get_services();
			std::vector<iop::DiscoveryServiceDef>::iterator itsvs;
			// add services
			for (itsvs = services.begin(); itsvs != services.end(); ++itsvs) {
				fkie_iop_msgs::msg::Service ros_srv;
				ros_srv.uri = itsvs->service_uri;
				ros_srv.major_version = itsvs->major_version;
				ros_srv.minor_version = itsvs->minor_version;
				ros_cmpt.services.push_back(ros_srv);
			}
			ros_node.components.push_back(ros_cmpt);
		}
		system.subsystems.push_back(ss_new);
	}
	p_pub_system->publish(system);
}

bool DiscoveryRosInterface::p_equal2ident(fkie_iop_msgs::msg::Identification &ros_ident, JausAddress &addr)
{
	if (ros_ident.address.subsystem_id == addr.getSubsystemID()
			&& ros_ident.address.node_id == addr.getNodeID()) {
		// do not compare the component ID:  ros_ident.address.component_id == addr.getComponentID()
		return true;
	}
	return false;
}

fkie_iop_msgs::msg::Node& DiscoveryRosInterface::p_get_node(fkie_iop_msgs::msg::Subsystem& ros_subsystem, JausAddress& addr)
{
	for (unsigned int i = 0; i < ros_subsystem.nodes.size(); i++) {
		fkie_iop_msgs::msg::Node& node = ros_subsystem.nodes[i];
		if (p_equal2ident(node.ident, addr)) {
			return node;
		}
	}
	// create new one
	fkie_iop_msgs::msg::Node node;
	std::map<JausAddress, fkie_iop_msgs::msg::Identification>::iterator itidnd = p_node_idents.find(p_get_node_addr(addr));
	if (itidnd != p_node_idents.end()) {
		node.ident = itidnd->second;
	} else {
		node.ident.address = p_convert(addr);
		p_node_idents[p_get_node_addr(addr)] = node.ident;
	}
	ros_subsystem.nodes.push_back(node);
	return ros_subsystem.nodes[ros_subsystem.nodes.size() - 1];

}

JausAddress DiscoveryRosInterface::p_get_node_addr(JausAddress& addr)
{
	return JausAddress(addr.getSubsystemID(), addr.getNodeID(), 255);
}

fkie_iop_msgs::msg::JausAddress DiscoveryRosInterface::p_convert(JausAddress& addr)
{
	fkie_iop_msgs::msg::JausAddress result;
	result.subsystem_id = addr.getSubsystemID();
	result.node_id = addr.getNodeID();
	result.component_id = addr.getComponentID();
	return result;
}
