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

#include "DiscoveryClientPlugin.h"
#include <pluginlib/class_list_macros.h>
#include <iop_component_fkie/iop_component.h>
#include "urn_jaus_jss_core_Discovery/DiscoveryService.h"

using namespace iop;
using namespace urn_jaus_jss_core_Discovery;
using namespace urn_jaus_jss_core_DiscoveryClient;
using namespace urn_jaus_jss_core_EventsClient;
using namespace urn_jaus_jss_core_Transport;


DiscoveryClientPlugin::DiscoveryClientPlugin()
{
	p_my_service = NULL;
	p_base_service = NULL;
	p_transport_service = NULL;
}

JTS::Service* DiscoveryClientPlugin::get_service()
{
	return p_my_service;
}

void DiscoveryClientPlugin::create_service(JTS::JausRouter* jaus_router)
{
	p_base_service = static_cast<EventsClientService *>(get_base_service());
	p_transport_service = static_cast<TransportService *>(get_base_service(2));
	p_my_service = new DiscoveryClientService(jaus_router, p_transport_service, p_base_service);
}

void DiscoveryClientPlugin::init_service()
{
	iop::Component &cmp = iop::Component::get_instance();
	DiscoveryService *discovery_srv = static_cast<DiscoveryService*>(cmp.get_service("Discovery"));
	if (discovery_srv != NULL) {
		p_my_service->pDiscoveryClient_ReceiveFSM->setDiscoveryFSM(discovery_srv->pDiscovery_ReceiveFSM);
	}
}

void DiscoveryClientPlugin::register_service(PluginInterface *plugin)
{
	if (plugin != NULL) {
		p_my_service->pDiscoveryClient_ReceiveFSM->appendServiceUri(plugin->get_service_uri(), plugin->get_version_number_major(), plugin->get_version_number_minor());
	}
}

PLUGINLIB_EXPORT_CLASS(iop::DiscoveryClientPlugin, iop::PluginInterface)
