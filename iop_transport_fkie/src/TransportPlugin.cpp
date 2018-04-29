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

#include <pluginlib/class_list_macros.h>

#include "TransportPlugin.h"

using namespace iop;
using namespace urn_jaus_jss_core_Transport;


TransportPlugin::TransportPlugin()
{
	p_my_service = NULL;
}

JTS::Service* TransportPlugin::get_service()
{
	return p_my_service;
}

void TransportPlugin::create_service(JTS::JausRouter* jaus_router)
{
	p_my_service = new TransportService(jaus_router);
}

PLUGINLIB_EXPORT_CLASS(iop::TransportPlugin, iop::PluginInterface)
