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


#ifndef DISCOVERY_RECEIVEFSM_H
#define DISCOVERY_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_core_Discovery/Messages/MessageSet.h"
#include "urn_jaus_jss_core_Discovery/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "Discovery_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include <fkie_iop_discovery/DiscoveryComponentList.h>
#include <fkie_iop_discovery/DiscoveryServiceDef.h>


#define RS_SSList ReportServiceList::Body::SubsystemList
#define RS_NList  RS_SSList::SubsystemSeq::NodeList
#define RS_CList  RS_NList::NodeSeq::ComponentList

namespace urn_jaus_jss_core_Discovery
{

const int TYPE_SYSTEM = 1;
const int TYPE_SUBSYSTEM = 2;
const int TYPE_NODE = 3;
const int TYPE_COMPONENT = 4;

class DllExport Discovery_ReceiveFSM : public JTS::StateMachine
{
public:
	Discovery_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~Discovery_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void publishServicesAction(RegisterServices msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportConfigurationAction(QueryConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportIdentificationAction(QueryIdentification msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportServiceListAction(QueryServiceList msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportServicesAction(QueryServices msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportSubsystemListAction(QuerySubsystemList msg, Receive::Body::ReceiveRec transportData);

	/// Guard Methods
	/// User Methods
	void registerService(std::string serviceuri, unsigned char maxver, unsigned char minver, JausAddress address);
	void registerSubsystem(JausAddress address);
	uint8_t getSystemID() { return system_id; }
	std::vector<iop::DiscoveryComponent> getComponents(std::string uri);

	Discovery_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;

	/** Variables used for registration by system **/
	std::vector<JausAddress> p_subsystems;
	iop::DiscoveryComponentList p_component_list;
	JausAddress p_own_address;

	// ros parameter
	// 0: Reserved, 1: System Identification, 2: Subsystem Identification, 3: Node Identification, 4: Component Identification, 5 - 255: Reserved
	uint8_t system_id;
	// 10001: VEHICLE, 20001: OCU, 30001: OTHER_SUBSYSTEM, 40001: NODE, 50001: PAYLOAD, 60001: COMPONENT
	uint16_t system_type;
	std::string name_subsystem;
	std::string name_node;
	int64_t p_timeout_lost;
	// In case the component with discvery service was restarted we have to ensure other components re-regsiter their services
	std::map<JausAddress, unsigned long> p_respond_ident;

	bool isComponentRequested(QueryServices &msg, unsigned int nodeid, unsigned int compid);
	bool isComponentRequested(QueryServiceList &msg, unsigned int subsystemid, unsigned int nodeid, unsigned int compid);
	RS_SSList::SubsystemSeq *p_add_subsystem(RS_SSList *list, unsigned int id);
	RS_NList::NodeSeq *p_add_node(RS_NList *list, unsigned int id);
	RS_CList::ComponentSeq *p_add_component(RS_CList *list, unsigned int id);
	std::map<uint8_t, std::string> system_id_map();
	std::map<uint16_t, std::string> system_type_map();
	// this method is to inform other components about restart of this component by send service list on second query identification without request services
	bool p_should_send_services(JausAddress address);
};

}

#endif // DISCOVERY_RECEIVEFSM_H
