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
#include <class_loader/multi_library_class_loader.hpp>
#include <fkie_iop_component/iop_config.h>
#include <fkie_iop_component/iop_plugin_interface.h>
#include "EventReceiver.h"
#include "Transport/JausTransport.h"
#include "InternalEvents/InternalEvent.h"

namespace iop
{
	class Component;
	class IopJausRouter: public JTS::JausRouter
	{
	public:
		IopJausRouter(JausAddress jausAddress, JTS::InternalEventHandler* ieHandler, std::string config, Component* component) : JTS::JausRouter(jausAddress, ieHandler, config), p_component(component) {}
		~IopJausRouter() {}

		Component* getComponent() { return p_component; }
	protected:
		Component* p_component;
	};

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

		bool has_service(std::string service_uri);
		void start_component();
		void shutdown_component();
		// level: 0-OK, 1-WARNING, 2-ERROR, 3 STALE
		void send_diagnostic(int level, std::string message);

		JTS::Service* get_service(std::string service_name);
	protected:

		struct ServiceInfo {
			ServiceInfo(JTS::Service* si, std::string uri) {
				this->service = si;
				this->uri = uri;
			}
			ServiceInfo() {
				this->service = 0;
				this->uri = "";
			}
			JTS::Service* service;
			std::string uri;
		};

		static Component* global_ptr;
		virtual void processInternalEvent(JTS::InternalEvent* ie);

		JausAddress p_own_address;
		boost::shared_ptr<iop::PluginInterface> p_discovery_client;
		std::map<std::string, std::string > p_service_package_list;
		pluginlib::ClassLoader<iop::PluginInterface>* p_class_loader;
		std::map<std::string, boost::shared_ptr<iop::PluginInterface> > p_plugins_map;
		std::map<std::string, boost::shared_ptr<iop::PluginInterface> > p_plugins_empty;
		std::map<std::string, iop::PluginInterface::ServiceInfo> p_cache_service_info;
		std::vector<ServiceInfo> service_list;
		IopJausRouter* jausRouter;
		std::string p_config_path;
		boost::thread *p_connect_thread;
		ros::NodeHandle p_pnh;
		ros::Publisher p_publisher_diagnostics;

		void p_connect_2_rte();
		void load_plugins();
		boost::shared_ptr<iop::PluginInterface> p_init_plugin(std::string name, pluginlib::ClassLoader<iop::PluginInterface>& class_loader);
		iop::PluginInterface::ServiceInfo p_read_service_info(std::string serviceid, std::string manifest);

		std::string p_get_plugin_manifest(std::string plugin_name);
		void p_read_manifests(pluginlib::ClassLoader<iop::PluginInterface>& class_loader);
		std::string p_uri_to_name(std::string service_uri);

	};
};
#endif // IOP_COMPONENT_H
