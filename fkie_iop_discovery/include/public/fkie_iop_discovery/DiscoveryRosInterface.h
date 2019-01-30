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


#ifndef DISCOVERY_ROS_INTERFACE_H
#define DISCOVERY_ROS_INTERFACE_H

#include <ros/ros.h>
#include <fkie_iop_msgs/Identification.h>
#include <fkie_iop_msgs/Component.h>
#include <fkie_iop_msgs/QueryIdentification.h>
#include <fkie_iop_msgs/JausAddress.h>
#include <fkie_iop_msgs/Node.h>
#include <fkie_iop_msgs/Service.h>
#include <fkie_iop_msgs/Subsystem.h>
#include <fkie_iop_msgs/System.h>
#include <std_srvs/Empty.h>

#include <fkie_iop_discovery/DiscoveryComponent.h>
#include "urn_jaus_jss_core_DiscoveryClient/Messages/MessageSet.h"
#include "JTSStateMachine.h"
#include "Transport/JausTransport.h"
#include "DiscoveryComponentList.h"
#include "DiscoveryServiceList.h"

namespace iop
{

typedef urn_jaus_jss_core_DiscoveryClient::ReportServiceList::Body::SubsystemList::SubsystemSeq::NodeList::NodeSeq::ComponentList::ComponentSeq::ServiceList ServiceListV1;
typedef urn_jaus_jss_core_DiscoveryClient::ReportServices::Body::NodeList::NodeSeq::ComponentList::ComponentSeq::ServiceList ServiceListV0;

class DiscoveryRosInterface {
public:
	DiscoveryRosInterface();
	void setup(JTS::StateMachine& jaus_router);
	/** Return true, if services should be updated. */
	bool update_ident(JausAddress &addr, urn_jaus_jss_core_DiscoveryClient::ReportIdentification &report_ident);
	void update_services(JausAddress discovery_addr, urn_jaus_jss_core_DiscoveryClient::ReportServiceList msg);
	void update_services(JausAddress discovery_addr, urn_jaus_jss_core_DiscoveryClient::ReportServices msg);
//	bool add_service(std::string service_uri, unsigned char major_version, unsigned char minor_version=255);
	void set_discovery_timeout(int timeout);
	bool enabled();

protected:
	JTS::StateMachine* p_jaus_router;
	/** Parameter and functions for ROS interface to publish the IOP system.*/
	bool p_enable_ros_interface;
	int p_force_component_update_after;
	int p_timeout_discover_service;
	ros::Publisher p_pub_identification;
	ros::Publisher p_pub_system;
	ros::ServiceServer p_srv_query_ident;
	ros::ServiceServer p_srv_update_system;

	iop::DiscoveryComponentList p_components;
	std::map<JausAddress, fkie_iop_msgs::Identification> p_subsystem_idents;
	std::map<JausAddress, fkie_iop_msgs::Identification> p_node_idents;
	std::map<JausAddress, unsigned int> p_discovery_srvs_stamps;  // subsystem ID, seconds of last update

	bool pQueryIdentificationSrv(fkie_iop_msgs::QueryIdentification::Request  &req, fkie_iop_msgs::QueryIdentification::Response &res);
	bool pUpdateSystemSrv(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
	void p_publish_subsystem();
	void p_query_identification(int query_type, jUnsignedShortInteger subsystem, jUnsignedByte node, jUnsignedByte component);
	bool p_equal2ident(fkie_iop_msgs::Identification& ros_ident, JausAddress& addr);

	fkie_iop_msgs::Node& p_get_node(fkie_iop_msgs::Subsystem& ros_subsystem, JausAddress& addr);
	JausAddress p_get_node_addr(JausAddress& addr);
	fkie_iop_msgs::JausAddress p_convert(JausAddress& addr);
};

};

#endif
