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

#include <exception>
#include "iop_component_fkie/iop_component.h"
#include "JausUtils.h"

using namespace JTS;
using namespace iop;

Component* Component::global_ptr = 0;

Component::Component(unsigned int subsystem, unsigned short node, unsigned short component)
{
	global_ptr = this;
	this->jausRouter = new JausRouter(JausAddress(subsystem, node, component), ieHandler);
	p_class_loader = NULL;
	p_pnh = ros::NodeHandle("~");
	load_plugins();
}

Component::~Component()
{
	boost::shared_ptr<iop::PluginInterface> plugin;
	while (!p_plugins.empty())
	{
		plugin = p_plugins.back();
		p_plugins.pop_back();
		plugin.reset();
	}

	Service* service;

	while (!serviceList.empty())
	{
		service = serviceList.back();
		serviceList.pop_back();

		delete service;
	}

	delete p_class_loader;
	delete jausRouter;
}


void Component::load_plugins()
{
	boost::shared_ptr<iop::PluginInterface> discovery_client;
	std::vector<std::pair<std::string, std::string> > iop_services;
	std::vector<std::string> p_servicelist;
	std::vector<std::string> p_xml_paths;
	XmlRpc::XmlRpcValue v;
	p_pnh.param("services", v, v);
	ROS_INFO("Load IOP plugin services specified by ~services parameter:");
	for(int i = 0; i < v.size(); i++) {
		if (v[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
			for(XmlRpc::XmlRpcValue::ValueStruct::iterator iterator = v[i].begin(); iterator != v[i].end(); iterator++) {
				std::string package = iterator->first;
				std::string service = iterator->second;
				iop_services.push_back(std::make_pair(package, service));
				p_servicelist.push_back(service);
				ros::package::getPlugins(package, "plugin", p_xml_paths);
			}
		}
	}

	p_class_loader = new pluginlib::ClassLoader<iop::PluginInterface>("iop_component_fkie", "iop::PluginInterface", std::string("plugin"), p_xml_paths);
	std::vector<std::string> classes = p_class_loader->getDeclaredClasses();
	for(unsigned int c = 0; c < classes.size(); ++c)
	{
		ROS_DEBUG("  Available classes: %s", classes[c].c_str());
	}
	try
	{
		for (unsigned int i = 0; i < p_servicelist.size(); ++i)
		{
			ROS_INFO("  loading %s ...", p_servicelist[i].c_str());
			boost::shared_ptr<iop::PluginInterface> plugin = p_class_loader->createInstance(p_servicelist[i]);
			if (plugin->get_base_service_type() == typeid(NULL)) {
				plugin->create_jts_service(this->jausRouter);
				JTS::Service* iop_service = plugin->get_iop_service();
				ROS_DEBUG("Initialized IOP-plugin for %s", plugin->get_iop_service_type().name());
				serviceList.push_back(iop_service);
				if (plugin->is_discovery_client()) {
					discovery_client = plugin;
				}
			}
			p_plugins.push_back(plugin);
		}
		bool one_initialized;
		do
		{
			one_initialized = false;
			for (unsigned int i = 0; i < p_plugins.size(); ++i)
			{
				if (p_plugins[i]->get_iop_service() == NULL) {
					try
					{
						boost::shared_ptr<iop::PluginInterface> base_plugin = p_get_plugin(p_plugins[i]->get_base_service_type());
						if (base_plugin != NULL) {
							p_plugins[i]->set_base_plugin(base_plugin.get());
							p_plugins[i]->create_jts_service(this->jausRouter);
							JTS::Service* iop_service = p_plugins[i]->get_iop_service();
							ROS_DEBUG("Initialized IOP-plugin for %s, with base service %s", p_plugins[i]->get_iop_service_type().name(), base_plugin->get_iop_service_type().name());
							serviceList.push_back(iop_service);
							one_initialized = true;
							if (p_plugins[i]->is_discovery_client()) {
								discovery_client = p_plugins[i];
							}
						}
					} catch (std::runtime_error &rex){
						throw rex;
					} catch (std::logic_error &no_plugin){
						// the neede plugin is still not available, perhaps in the next iteration
						// if not, it end if one_initialized was not set to true
					}
				}
			}
		} while (one_initialized);
	}
	catch(pluginlib::PluginlibException& ex)
	{
		throw std::runtime_error("The plugin failed to load for some reason. Error: " + std::string(ex.what()));
	}
	for (unsigned int i = 0; i < p_plugins.size(); ++i)
	{
		if (p_plugins[i]->get_iop_service() == NULL) {
			throw std::runtime_error("Can not initialize plugin for " +
					std::string(p_plugins[i]->get_iop_service_type().name()) +
					". Plugin for base service " +
					std::string(p_plugins[i]->get_base_service_type().name()) + " not found!");
		}
	}
	ROS_INFO("... plugin loading complete!");
	// register the services
	if (discovery_client != NULL) {
		ROS_INFO("Register services by discovery service...");
		for (unsigned int i = 0; i < p_plugins.size(); ++i)
		{
			discovery_client->register_service(p_plugins[i].get());
		}
		ROS_INFO("... register complete");
	} else {
		ROS_INFO("Discovery client not found, the services will not be registered!");
	}

	ROS_INFO("Initialize plugins ...");
	for (unsigned int i = 0; i < p_plugins.size(); ++i)
	{
		p_plugins[i]->init_jts_service();
	}
	ROS_INFO("... initialization complete");
}


boost::shared_ptr<iop::PluginInterface> Component::p_get_plugin(const std::type_info &iop_service)
{
	for (unsigned int i = 0; i < p_plugins.size(); ++i)
	{
		if (p_plugins[i]->get_iop_service_type() == iop_service
				&& p_plugins[i]->get_iop_service() != NULL) {
			return p_plugins[i];
		}
	}
	throw std::logic_error("plugin is not available");
}


void Component::start_component()
{
	Service* service;

	jausRouter->start();
	this->start();

	for (unsigned int i = 0; i < serviceList.size(); i++)
	{
		 service = serviceList.at(i);
		 service->start();
	}

}


void Component::shutdown_component()
{
	Service* service;

	for (unsigned int i = 0; i < serviceList.size(); i++)
	{
		 service = serviceList.at(i);
		 service->stop();
	}

	this->stop();
	jausRouter->stop();
}


JTS::Service* Component::get_service(const std::type_info &iop_service)
{
	for (unsigned int i = 0; i < p_plugins.size(); ++i)
	{
		if (p_plugins[i]->get_iop_service_type() == iop_service
				&& p_plugins[i]->get_iop_service() != NULL) {
			return p_plugins[i]->get_iop_service();
		}
	}
	return NULL;
}


void Component::processInternalEvent(InternalEvent *ie)
{
	bool done = false;

	//
	// When a component receives an internal event, it passes it
	// to the services to handling, children services first.  If the
	// event is not processed by normal transitions, it's passed
	// again to the services (children first) for default transitions.
	// A given event may only be processed by at most one service.
	//
	for (unsigned int i = serviceList.size(); i>0; i--)
	{
		if (!done) done = serviceList.at(i-1)->processTransitions(ie);
	}
	for (unsigned int i = serviceList.size(); i>0; i--)
	{
		if (!done) done = serviceList.at(i-1)->defaultTransitions(ie);
	}
}

