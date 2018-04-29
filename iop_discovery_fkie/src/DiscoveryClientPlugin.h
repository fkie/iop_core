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


#ifndef DISCOVERYCLIENTPLUGIN_H
#define DISCOVERYCLIENTPLUGIN_H

#include "urn_jaus_jss_core_DiscoveryClient/DiscoveryClientService.h"
#include "urn_jaus_jss_core_EventsClient/EventsClientService.h"
#include "urn_jaus_jss_core_Transport/TransportService.h"

#include <iop_component_fkie/iop_plugin_interface.h>

namespace iop
{

class DllExport DiscoveryClientPlugin : public PluginInterface
{
public:
	DiscoveryClientPlugin();

	JTS::Service* get_service();
	void create_service(JTS::JausRouter* jaus_router);
	void init_service();

	bool is_discovery_client() { return true; }
	void register_service(PluginInterface *plugin);

protected:
	urn_jaus_jss_core_DiscoveryClient::DiscoveryClientService *p_my_service;
	urn_jaus_jss_core_EventsClient::EventsClientService *p_base_service;
	urn_jaus_jss_core_Transport::TransportService *p_transport_service;

};

};

#endif
