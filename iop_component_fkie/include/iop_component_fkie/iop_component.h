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


#ifndef IOP_COMPONENT_H
#define IOP_COMPONENT_H

#include <vector>
#include <map>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <class_loader/multi_library_class_loader.h>
#include <iop_component_fkie/iop_plugin_interface.h>
#include "EventReceiver.h"
#include "Transport/JausTransport.h"
#include "InternalEvents/InternalEvent.h"

namespace iop
{
	class Component : public JTS::EventReceiver
	{
	public:
		static Component& get_instance()
		{
			if (global_ptr == 0) throw std::runtime_error("Component was not initialized!");
			return *global_ptr;
		}
		Component(unsigned int subsystem, unsigned short node, unsigned short component);
		virtual ~Component();

		void start_component();
		void shutdown_component();

		JTS::Service* get_service(const std::type_info &iop_service);
	protected:
		static Component* global_ptr;
		virtual void processInternalEvent(JTS::InternalEvent* ie);

		pluginlib::ClassLoader<iop::PluginInterface>* p_class_loader;
		std::vector<boost::shared_ptr<iop::PluginInterface> > p_plugins;
		std::vector<JTS::Service*> serviceList;
		JTS::JausRouter* jausRouter;
		ros::NodeHandle p_pnh;

		void load_plugins();
		boost::shared_ptr<iop::PluginInterface> p_get_plugin(const std::type_info &iop_service);

	};
};
#endif // IOP_COMPONENT_H
