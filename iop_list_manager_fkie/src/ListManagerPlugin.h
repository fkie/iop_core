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


#ifndef LISTMANAGERPLUGIN_H
#define LISTMANAGERPLUGIN_H

#include "urn_jaus_jss_core_ListManager/ListManagerService.h"
#include "urn_jaus_jss_core_AccessControl/AccessControlService.h"
#include "urn_jaus_jss_core_Events/EventsService.h"
#include "urn_jaus_jss_core_Management/ManagementService.h"
#include "urn_jaus_jss_core_Transport/TransportService.h"

#include <iop_component_fkie/iop_plugin_interface.h>

namespace iop
{

class DllExport ListManagerPlugin : public PluginInterface
{
public:
	ListManagerPlugin();

	JTS::Service* get_service();
	void create_service(JTS::JausRouter* jaus_router);

protected:
	urn_jaus_jss_core_ListManager::ListManagerService *p_my_service;
	urn_jaus_jss_core_Management::ManagementService *p_base_service;
	urn_jaus_jss_core_AccessControl::AccessControlService *p_accesscontrol_service;
	urn_jaus_jss_core_Events::EventsService *p_events_service;
	urn_jaus_jss_core_Transport::TransportService *p_transport_service;

};

};

#endif
