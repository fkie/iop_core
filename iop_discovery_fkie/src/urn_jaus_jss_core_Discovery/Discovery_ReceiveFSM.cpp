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


#include "urn_jaus_jss_core_Discovery/Discovery_ReceiveFSM.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <string.h>
#include <algorithm>
#include <iop_component_fkie/iop_config.h>


using namespace JTS;

namespace urn_jaus_jss_core_Discovery
{

Discovery_ReceiveFSM::Discovery_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new Discovery_ReceiveFSMContext(*this);
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	system_id = 4;
	system_type = 6001;
	name_subsystem = "Robotname";
	name_node = "Componentname";
	p_timeout_lost = 60;
}

Discovery_ReceiveFSM::~Discovery_ReceiveFSM()
{
	delete context;
}

void Discovery_ReceiveFSM::registerService(int minver, int maxver, std::string serviceuri, JausAddress address)
{
	bool result = p_component_list.add_service(p_own_address, address, serviceuri, maxver, minver);
	if (result) {
		ROS_INFO_NAMED("Discovery", "registered own service '%s' [%s]", serviceuri.c_str(), address.str().c_str());
	} else {
		ROS_WARN_NAMED("Discovery", "own service '%s' [%s] already exists, ignore", serviceuri.c_str(), address.str().c_str());
	}
}

void Discovery_ReceiveFSM::registerSubsystem(JausAddress address)
{
	bool discovered = false;
	for (int i=1; i<p_subsystems.size(); i++) {
		if (address.getSubsystemID() == p_subsystems[i].getSubsystemID()) {
			discovered = true;
			break;
		}
	}
	if (!discovered) {
		p_subsystems.push_back(address);
	}
}

void Discovery_ReceiveFSM::setupNotifications()
{
	pEvents_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_Discovery_ReceiveFSM_Receiving_Ready", "Events_ReceiveFSM");
	pEvents_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_Discovery_ReceiveFSM_Receiving_Ready", "Events_ReceiveFSM");
	registerNotification("Receiving_Ready", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving_Ready", "Discovery_ReceiveFSM");
	registerNotification("Receiving", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving", "Discovery_ReceiveFSM");
	iop::Config cfg("~Discovery");
	cfg.param("system_id", system_id, system_id, true, true, "", system_id_map());
	cfg.param("system_type", system_type, system_type, true, true, "", system_type_map());
	cfg.param("name_subsystem", name_subsystem, name_subsystem);
	cfg.param("name_node", name_node, name_node);
	cfg.param("timeout_lost", p_timeout_lost, p_timeout_lost);
	p_own_address = *(this->jausRouter->getJausAddress());
	p_component_list.set_timeout(p_timeout_lost);
}

std::map<int, std::string> Discovery_ReceiveFSM::system_id_map()
{
	std::map<int, std::string> result;
	result[1] = "System";
	result[2] = "Subsystem";
	result[3] = "Node";
	result[4] = "Component";
	return result;
}

std::map<int, std::string> Discovery_ReceiveFSM::system_type_map()
{
	std::map<int, std::string> result;
	result[10001] = "VEHICLE";
	result[20001] = "OCU";
	result[30001] = "OTHER_SUBSYSTEM";
	result[40001] = "NODE";
	result[50001] = "PAYLOAD";
	result[60001] = "COMPONENT";
	return result;
}

void Discovery_ReceiveFSM::publishServicesAction(RegisterServices msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	RegisterServices::RegisterServicesBody::ServiceList *services = msg.getRegisterServicesBody()->getServiceList();
	ROS_DEBUG_NAMED("Discovery", "Register %u new services...", services->getNumberOfElements());
	for (unsigned int i = 0; i < services->getNumberOfElements(); i++) {
		RegisterServices::RegisterServicesBody::ServiceList::ServiceRec *service = msg.getRegisterServicesBody()->getServiceList()->getElement(i);
		bool result = p_component_list.add_service(p_own_address, sender, service->getURI(), service->getMajorVersionNumber(), service->getMinorVersionNumber());
		if (result) {
			ROS_INFO_NAMED("Discovery", "registered '%s' [%s]", service->getURI().c_str(), sender.str().c_str());
		} else {
			ROS_WARN_NAMED("Discovery", "service '%s' [%s] already exists, ignore", service->getURI().c_str(), sender.str().c_str());
		}
	}
	p_component_list.update_ts(p_own_address, sender);
}

void Discovery_ReceiveFSM::sendReportConfigurationAction(QueryConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	// Extract the sender information
	JausAddress sender = transportData.getAddress();
	int query_type = msg.getBody()->getQueryConfigurationRec()->getQueryType();
	ROS_DEBUG_NAMED("Discovery", "sendReportConfiguration for query_type: %d, sender: %s", query_type, sender.str().c_str());

	ReportConfiguration report_msg;
	int cnt_nodes = 0;
	int cnt_cmps = 0;
	// define filter for requested configuration
	unsigned short subsystem = (query_type < TYPE_SUBSYSTEM) ? 65535 : sender.getSubsystemID();
	unsigned char node = (query_type < TYPE_NODE) ? 255 : sender.getNodeID();
	unsigned char component = (query_type < TYPE_COMPONENT) ? 255 : sender.getComponentID();
	std::vector<iop::DiscoveryComponent> components = p_component_list.get_components(p_own_address, subsystem, node, component);
	std::vector<iop::DiscoveryComponent>::iterator itcmp;
	for (itcmp = components.begin(); itcmp != components.end(); itcmp++) {
		bool cmp_added = false;
		ReportConfiguration::Body::NodeList *node_list = report_msg.getBody()->getNodeList();
		for (unsigned int jn = 0; jn < node_list->getNumberOfElements(); jn++) {
			ReportConfiguration::Body::NodeList::NodeSeq *node_seq = node_list->getElement(jn);
			if (itcmp->address.getNodeID() == node_seq->getNodeRec()->getNodeID()) {
				ReportConfiguration::Body::NodeList::NodeSeq::ComponentList *cmp_list = node_seq->getComponentList();
				for (unsigned int jc = 0; jc < cmp_list->getNumberOfElements(); jc++) {
					ReportConfiguration::Body::NodeList::NodeSeq::ComponentList::ComponentRec *cmp_rec = cmp_list->getElement(jc);
					if (itcmp->address.getComponentID() == cmp_rec->getComponentID()) {
						cmp_added = true;
					}
				}
				if (!cmp_added) {
					ReportConfiguration::Body::NodeList::NodeSeq::ComponentList::ComponentRec cmp;
					cmp.setComponentID(itcmp->address.getComponentID());
					node_seq->getComponentList()->addElement(cmp);
					cmp_added = true;
					cnt_cmps++;
				}
			}
		}
		if (!cmp_added) {
			ReportConfiguration::Body::NodeList::NodeSeq::ComponentList::ComponentRec cmp;
			cmp.setComponentID(itcmp->address.getComponentID());
			ReportConfiguration::Body::NodeList::NodeSeq node;
			node.getNodeRec()->setNodeID(itcmp->address.getNodeID());
			node.getComponentList()->addElement(cmp);
			node_list->addElement(node);
			cmp_added = true;
			cnt_nodes++;
			cnt_cmps++;
		}
	}
	ROS_DEBUG_NAMED("Discovery", "	report configuration with %d nodes and %d components", cnt_nodes, cnt_cmps);
	sendJausMessage( report_msg, sender);
}

void Discovery_ReceiveFSM::sendReportIdentificationAction(QueryIdentification msg, Receive::Body::ReceiveRec transportData)
{
	int query_type = msg.getBody()->getQueryIdentificationRec()->getQueryType();
	if (system_id <= query_type) {
		JausAddress sender = transportData.getAddress();
		ReportIdentification report_msg;
		std::string name = "InvalidName";
		if (query_type == TYPE_COMPONENT) {
			name = ros::this_node::getName();
			std::size_t pos = name.find_last_of("/");
			if (pos != std::string::npos) {
				name.replace(0, pos+1, "");
			}
		} else if (query_type == TYPE_NODE)	{
			name = name_node;
			std::size_t pos = name.find_last_of("/");
			if (pos != std::string::npos) {
				name.replace(0, pos+1, "");
			}
		} else if (query_type == TYPE_SUBSYSTEM) {
			name = name_subsystem;
		} else if (query_type == TYPE_SYSTEM) {
			name = "System";
		}
		ROS_DEBUG_NAMED("Discovery", "sendReportIdentification to %s: query_type: %d, system_type: %d, name: %s",
				sender.str().c_str(),query_type, system_type, name.c_str());
		report_msg.getBody()->getReportIdentificationRec()->setQueryType(query_type);
		report_msg.getBody()->getReportIdentificationRec()->setType(system_type);
		report_msg.getBody()->getReportIdentificationRec()->setIdentification(name);
		sendJausMessage(report_msg, sender);
		p_component_list.update_ts(p_own_address, sender);
	} else {
		ROS_WARN_ONCE_NAMED("Discovery", "sendReportIdentification own system_id_type: %d>%d (query_type), do not response", system_id, query_type);
	}
}

void Discovery_ReceiveFSM::sendReportServiceListAction(QueryServiceList msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	if (system_id == TYPE_SUBSYSTEM) {
		ROS_DEBUG_NAMED("Discovery", "sendReportServiceList to %s", sender.str().c_str());
		ReportServiceList report_msg;
		int cnt_services = 0;
		std::vector<iop::DiscoveryComponent> components = p_component_list.get_components(p_own_address);
		std::vector<iop::DiscoveryComponent>::iterator itcmp;
		for (itcmp = components.begin(); itcmp != components.end(); itcmp++) {
			JausAddress addr = itcmp->address;
			if (isComponentRequested(msg, addr.getSubsystemID(), addr.getNodeID(), addr.getComponentID())) {
				std::vector<iop::DiscoveryServiceDef> services = itcmp->get_services();
				std::vector<iop::DiscoveryServiceDef>::iterator itsrv;
				for (itsrv = services.begin(); itsrv != services.end(); itsrv++) {
					bool service_added = false;
					ReportServiceList::Body::SubsystemList *sslist = report_msg.getBody()->getSubsystemList();
					ReportServiceList::Body::SubsystemList::SubsystemSeq *ssrec = p_add_subsystem(sslist, addr.getSubsystemID());
					ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq *nrec = p_add_node(ssrec->getNodeList(), addr.getNodeID());
					ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentSeq *crec = p_add_component(nrec->getComponentList(), addr.getComponentID());
					ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentSeq::ServiceList::ServiceRec srvrec;
					srvrec.setURI(itsrv->service_uri);
					srvrec.setMajorVersionNumber(itsrv->major_version);
					srvrec.setMinorVersionNumber(itsrv->minor_version);
					crec->getServiceList()->addElement(srvrec);
					cnt_services++;
				}
			}
		}
		ROS_DEBUG_NAMED("Discovery", "	report services with %d services", cnt_services);
		sendJausMessage( report_msg, sender );
	} else {
		ROS_WARN_ONCE_NAMED("Discovery", "ignore QueryServiceList from %s, since own system_id is not SYSTEM(1)", sender.str().c_str());
	}
}

bool Discovery_ReceiveFSM::isComponentRequested(QueryServiceList &msg, unsigned int subsystemid, unsigned int nodeid, unsigned int compid)
{
	QueryServiceList::Body::SubsystemList *sslist = msg.getBody()->getSubsystemList();
	unsigned int count_ssystems = sslist->getNumberOfElements();
	if (count_ssystems < 1) {
		return true;
	}
	for (unsigned int s = 0; s < count_ssystems; s++) {
		QueryServiceList::Body::SubsystemList::SubsystemSeq *ssreq = sslist->getElement(s);
		unsigned int ssid = ssreq->getSubsystemRec()->getSubsystemID();
		// check subsystem ID: 65535 -> all subsystem IDs
		if (ssid == 65535 || ssid == subsystemid) {
			QueryServiceList::Body::SubsystemList::SubsystemSeq::NodeList *nodelist = ssreq->getNodeList();
			// check node ID: 255 -> all node IDs
			unsigned int count_nodes = nodelist->getNumberOfElements();
			if (count_nodes < 1) {
				return true;
			}
			for (unsigned int n = 0; n < count_nodes; n++) {
				QueryServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq *nodesec = nodelist->getElement(n);
				unsigned int nid = nodesec->getNodeRec()->getNodeID();
				if (nid == 255 or nid == nodeid) {
					// check the component ID; 255 -> all components of the node
					QueryServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList *complist = nodesec->getComponentList();
					unsigned int count_comps = complist->getNumberOfElements();
					if (count_comps < 1) {
						return true;
					}
					for (unsigned int c = 0; c < count_comps; c++) {
						unsigned int cid = complist->getElement(c)->getComponentID();
						if (cid == 255 or cid == compid) {
							return true;
						}
					}
				}
			}
		}
	}
	return false;
}

RS_SSList::SubsystemSeq *Discovery_ReceiveFSM::p_add_subsystem(RS_SSList *list, unsigned int id)
{
	RS_SSList::SubsystemSeq *result;
	for (unsigned int i = 0; i < list->getNumberOfElements(); i++) {
		result = list->getElement(i);
		if (result->getSubsystemRec()->getSubsystemID() == id) {
			return result;
		}
	}
	RS_SSList::SubsystemSeq ssseq;
	ssseq.getSubsystemRec()->setSubsystemID(id);
	list->addElement(ssseq);
	return list->getElement(list->getNumberOfElements()-1);
}

RS_NList::NodeSeq *Discovery_ReceiveFSM::p_add_node(RS_NList *list, unsigned int id)
{
	RS_NList::NodeSeq *result;
	for (unsigned int i = 0; i < list->getNumberOfElements(); i++) {
		result = list->getElement(i);
		if (result->getNodeRec()->getNodeID() == id) {
			return result;
		}
	}
	RS_NList::NodeSeq nseq;
	nseq.getNodeRec()->setNodeID(id);
	list->addElement(nseq);
	return list->getElement(list->getNumberOfElements()-1);
}

RS_CList::ComponentSeq *Discovery_ReceiveFSM::p_add_component(RS_CList *list, unsigned int id)
{
	RS_CList::ComponentSeq *result;
	for (unsigned int i = 0; i < list->getNumberOfElements(); i++) {
		result = list->getElement(i);
		if (result->getComponentRec()->getComponentID() == id) {
			return result;
		}
	}
	RS_CList::ComponentSeq cseq;
	cseq.getComponentRec()->setComponentID(id);
	list->addElement(cseq);
	return list->getElement(list->getNumberOfElements()-1);
}

void Discovery_ReceiveFSM::sendReportServicesAction(QueryServices msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	if (system_id == TYPE_SUBSYSTEM) {
		ROS_DEBUG_NAMED("Discovery", "sendReportServices to %s", sender.str().c_str());
		ReportServices report_msg;
		int cnt_nodes = 0;
		int cnt_cmps = 0;
		int cnt_srvs = 0;
		std::vector<iop::DiscoveryComponent> components = p_component_list.get_components(p_own_address);
		std::vector<iop::DiscoveryComponent>::iterator itcmp;
		for (itcmp = components.begin(); itcmp != components.end(); itcmp++) {
			JausAddress addr = itcmp->address;
			if (isComponentRequested(msg, addr.getNodeID(), addr.getComponentID())) {
				std::vector<iop::DiscoveryServiceDef> services = itcmp->get_services();
				std::vector<iop::DiscoveryServiceDef>::iterator itsrv;
				for (itsrv = services.begin(); itsrv != services.end(); itsrv++) {
					bool service_added = false;
					ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq::ServiceList::ServiceRec srvrec;
					srvrec.setURI(itsrv->service_uri);
					srvrec.setMajorVersionNumber(itsrv->major_version);
					srvrec.setMinorVersionNumber(itsrv->minor_version);
					cnt_srvs++;
					ReportServices::Body::NodeList *node_list = report_msg.getBody()->getNodeList();
					for (unsigned int jn = 0; jn < node_list->getNumberOfElements(); jn++) {
						ReportServices::Body::NodeList::NodeSeq *node_seq = node_list->getElement(jn);
						if (addr.getNodeID() == node_seq->getNodeRec()->getNodeID()) {
							ReportServices::Body::NodeList::NodeSeq::ComponentList *cmp_list = node_seq->getComponentList();
							for (unsigned int jc = 0; jc < cmp_list->getNumberOfElements(); jc++) {
								ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq *cmp_seq = cmp_list->getElement(jc);
								if (addr.getComponentID() == cmp_seq->getComponentRec()->getComponentID()) {
									cmp_seq->getServiceList()->addElement(srvrec);
									service_added = true;
								}
							}
							if (!service_added) {
								ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq cmp;
								cmp.getComponentRec()->setComponentID(addr.getComponentID());
								cmp.getServiceList()->addElement(srvrec);
								node_seq->getComponentList()->addElement(cmp);
								service_added = true;
								cnt_cmps++;
							}
						}
					}
					if (!service_added) {
						ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq cmp;
						cmp.getComponentRec()->setComponentID(addr.getComponentID());
						cmp.getServiceList()->addElement(srvrec);
						ReportServices::Body::NodeList::NodeSeq node;
						node.getNodeRec()->setNodeID(addr.getNodeID());
						node.getComponentList()->addElement(cmp);
						node_list->addElement(node);
						service_added = true;
						cnt_nodes++;
						cnt_cmps++;
					}
				}
			}
		}
		ROS_DEBUG_NAMED("Discovery", "	report services with %d nodes, %d components and %d services", cnt_nodes, cnt_cmps, cnt_srvs);
		sendJausMessage( report_msg, sender );
	} else {
		ROS_WARN_NAMED("Discovery", "ignore QueryServices from %s, since own system_id is not SYSTEM(1)", sender.str().c_str());
	}
}

bool Discovery_ReceiveFSM::isComponentRequested(QueryServices &msg, unsigned int nodeid, unsigned int compid)
{
	QueryServices::Body::NodeList *nodelist = msg.getBody()->getNodeList();
	// check node ID: 255 -> all node IDs
	unsigned int count_nodes = nodelist->getNumberOfElements();
	if (count_nodes < 1) {
		return true;
	}
	for (unsigned int n = 0; n < count_nodes; n++) {
		QueryServices::Body::NodeList::NodeSeq *nodesec = nodelist->getElement(n);
		unsigned int nid = nodesec->getNodeRec()->getNodeID();
		if (nid == 255 or nid == nodeid) {
			// check the component ID; 255 -> all components of the node
			QueryServices::Body::NodeList::NodeSeq::ComponentList *complist = nodesec->getComponentList();
			unsigned int count_comps = complist->getNumberOfElements();
			if (count_comps < 1) {
				return true;
			}
			for (unsigned int c = 0; c < count_comps; c++) {
				unsigned int cid = complist->getElement(c)->getComponentID();
				if (cid == 255 or cid == compid) {
					return true;
				}
			}
		}
	}
	return false;
}

void Discovery_ReceiveFSM::sendReportSubsystemListAction(QuerySubsystemList msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	if (system_id == TYPE_SYSTEM) {
		ROS_DEBUG_NAMED("Discovery", "sendReportSubsystemList to %s, known subsystems: %lu", sender.str().c_str(), p_subsystems.size());
		ReportSubsystemList report_msg;
		// add only the discovery services of a subsystems
		for (int i=0; i < p_subsystems.size(); i++) {
			ReportSubsystemList::Body::SubsystemList::SubsystemRec comp;
			comp.setSubsystemID(p_subsystems[i].getSubsystemID());
			comp.setNodeID(p_subsystems[i].getNodeID());
			comp.setComponentID(p_subsystems[i].getComponentID());
			report_msg.getBody()->getSubsystemList()->addElement(comp);
		}
		sendJausMessage( report_msg, sender );
	} else {
		ROS_WARN_NAMED("Discovery", "ignore QuerySubsystemList from %s, since own system_id is not SYSTEM(1)", sender.str().c_str());
	}
}

std::vector<iop::DiscoveryComponent> Discovery_ReceiveFSM::getComponents(std::string uri)
{
	return p_component_list.get_components(p_own_address, uri);
}

};
