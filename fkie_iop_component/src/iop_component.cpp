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
#include <stdio.h>
#include <tinyxml2.h>
#include "fkie_iop_component/iop_component.h"
#include "fkie_iop_component/string.hpp"
#include "JausUtils.h"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

using namespace JTS;
using namespace iop;


Component* Component::global_ptr = 0;

Component::Component(unsigned int subsystem, unsigned short node, unsigned short component, rclcpp::Node& rosnode)
{
	global_ptr = this;
	p_rosnode = &rosnode;
	rcl_interfaces::msg::ParameterDescriptor service_description;
	service_description.name = "iop_services";
	service_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
	service_description.description = "List of service to include into component.";
	service_description.read_only = true;
	service_description.additional_constraints = "package_name/service_name";
	p_rosnode->declare_parameter<std::string>("iop_services", "", service_description);
	p_own_address = JausAddress(subsystem, node, component);
	p_publisher_diagnostics = p_rosnode->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
	p_config_path = "nm.cfg";
	if (rosnode.has_parameter("jaus_config")) {
		// read configuration from private parameter
		rosnode.get_parameter("jaus_config", p_config_path);
	}
	if (p_config_path.compare("nm.cfg") == 0) {
		char cwd[1024];
		if (getcwd(cwd, sizeof(cwd)) != nullptr) {
			RCLCPP_INFO(p_rosnode->get_logger(), "JAUS configuration file: %s, pwd: %s", p_config_path.c_str(), cwd);
		} else {
			RCLCPP_INFO(p_rosnode->get_logger(), "JAUS configuration file: %s", p_config_path.c_str());
		}
	} else {
		RCLCPP_INFO(p_rosnode->get_logger(), "JAUS configuration file: %s", p_config_path.c_str());
	}
	p_class_loader = nullptr;
	jausRouter = nullptr;
	// spawn another thread
	p_connect_thread = new std::thread(std::bind(&Component::p_connect_2_rte, this));
}

Component::~Component()
{
	p_connect_thread->join();
	delete p_connect_thread;
	std::map<std::string, std::shared_ptr<iop::PluginInterface> >::iterator it;
	for (it = p_plugins_map.begin(); it != p_plugins_map.end(); ++it) {
		std::cout << "  delete service:" << it->second.get()->get_service_uri() << std::endl;
		delete it->second.get()->get_service();
	}
	p_discovery_client = std::shared_ptr<iop::PluginInterface>();
	p_plugins_empty.clear();
	p_plugins_map.clear();
	service_list.clear();
	delete p_class_loader;
	delete jausRouter;
	std::cout << "Shutdown component finished" << std::endl;
}

void Component::p_connect_2_rte()
{
	RCLCPP_INFO(p_rosnode->get_logger(), "Connect to JAUS nodeManager...");
	send_diagnostic(3, "Connecting to RTE");
	this->jausRouter = new IopJausRouter(p_own_address, ieHandler, p_config_path, this);
	while (rclcpp::ok() && ! jausRouter->isConnected()) {
		sleep(1);
		delete this->jausRouter;
		send_diagnostic(2, "Timeout, connecting to RTE");
		this->jausRouter = new IopJausRouter(p_own_address, ieHandler, p_config_path, this);
	}
	if (rclcpp::ok() && jausRouter->isConnected()) {
		RCLCPP_INFO(p_rosnode->get_logger(), "JAUS ID: %d", this->jausRouter->getJausAddress()->get());
		send_diagnostic(3, "Load Plug-Ins");
		load_plugins();
		start_component();
		send_diagnostic(0, "Started");
	}
}

void Component::send_diagnostic(int level, std::string message)
{
	diagnostic_msgs::msg::DiagnosticArray dmsg;
	dmsg.header.stamp = p_rosnode->now();
	diagnostic_msgs::msg::DiagnosticStatus diag;
	diag.level = level;
	diag.name = p_rosnode->get_fully_qualified_name();
	diag.message = message;
	dmsg.status.push_back(diag);
	p_publisher_diagnostics->publish(dmsg);
}

void Component::load_plugins()
{
	p_discovery_client = std::shared_ptr<iop::PluginInterface>();
	std::vector<std::string> plugin_names;
	bool has_range_sensor_service = false;
	bool has_visual_sensor_service = false;
	bool has_discovery_service = false;
	bool has_discovery_client_service = false;
	RCLCPP_INFO(p_rosnode->get_logger(), "Load IOP plugin services specified by ~services parameter:");
	std::vector<std::string> v;
	bool param_available = p_rosnode->get_parameter("iop_services", v);
    	if (param_available) {
		for(unsigned int i = 0; i < v.size(); i++) {
			//std::vector<std::string> pkg_srv;
			auto pkg_srv = iop::split(v[i], '/');
			if (pkg_srv.size() == 2) {
				std::string package = pkg_srv[0];
				std::string service = pkg_srv[1];
				if (service.compare("RangeSensor") == 0 or service.compare("RangeSensorClient") == 0) {
					has_range_sensor_service = true;
				}
				if (service.compare("VisualSensor") == 0 or service.compare("VisualSensorClient") == 0) {
					has_visual_sensor_service = true;
				}
				if (service.compare("Discovery") == 0) {
					has_discovery_service = true;
				}
				if (service.compare("DiscoveryClient") == 0) {
					has_discovery_client_service = true;
				}
				p_service_package_list[service] = package;
				plugin_names.push_back(service);
			} else {
				RCLCPP_WARN(p_rosnode->get_logger(), "skipped plugin entry '%s' because of invalid format", v[i]);
			}
		}
	} else {
        	std::string msg = "iop_services parameter not available!";
        	// msg += services_param.get_type_name();
        	throw std::runtime_error(msg.c_str());
	}
	if (has_discovery_service & has_discovery_client_service) {
		RCLCPP_WARN(p_rosnode->get_logger(), "In this version you do not need to include Discovery and DiscoverClient in the same component!");
	}
	if (has_range_sensor_service & has_visual_sensor_service) {
		throw std::logic_error("You can not use RangeSensor{Client} and VisualSensor{Client} in the same component, since they use the same GeometricProperties message type!");
	}
	// determine paths for xml files with plugin description
	std::vector<std::string> p_xml_paths;
	p_class_loader = new pluginlib::ClassLoader<iop::PluginInterface>("fkie_iop_component", "iop::PluginInterface", std::string("plugin"), p_xml_paths);
	p_read_manifests(*p_class_loader);
	try {
		for (unsigned int i = 0; i < plugin_names.size(); ++i) {
			p_init_plugin(plugin_names[i], *p_class_loader);
		}
	} catch(pluginlib::PluginlibException& ex) {
		throw std::runtime_error("The plugin failed to load for some reason. Error: " + std::string(ex.what()));
	}
	// test for uninitialized pugins
	std::map<std::string, std::shared_ptr<iop::PluginInterface> >::iterator it;
	for (it = p_plugins_map.begin(); it != p_plugins_map.end(); ++it) {
		if (it->second.get()->get_service() == NULL) {
			throw std::runtime_error(it->second.get()->error_message);
		}
	}
	RCLCPP_INFO(p_rosnode->get_logger(), "... plugin loading complete!");
	// register the services
	if (p_discovery_client != NULL) {
		RCLCPP_INFO(p_rosnode->get_logger(), "Add services to register by discovery service...");
		for (it = p_plugins_map.begin(); it != p_plugins_map.end(); ++it) {
			p_discovery_client->register_service(it->second.get());
		}
		RCLCPP_INFO(p_rosnode->get_logger(), "... all services added");
	} else {
		RCLCPP_INFO(p_rosnode->get_logger(), "Discovery client not found, the services will not be registered!");
	}
	RCLCPP_INFO(p_rosnode->get_logger(), "Initialize plugins ...");
	for (it = p_plugins_map.begin(); it != p_plugins_map.end(); ++it) {
		it->second.get()->init_service();
	}
	RCLCPP_INFO(p_rosnode->get_logger(), "... initialization complete");
}

std::shared_ptr<iop::PluginInterface> Component::p_init_plugin(std::string name, pluginlib::ClassLoader<iop::PluginInterface>& class_loader)
{
	if (name.empty()) {
		return std::shared_ptr<iop::PluginInterface>();
	}
	std::map<std::string, std::shared_ptr<iop::PluginInterface> >::iterator it = p_plugins_map.find(name);
	if (it != p_plugins_map.end()) {
		return it->second;
	} else {
		std::shared_ptr<iop::PluginInterface> plugin = class_loader.createSharedInstance(name);
		plugin->set_service_info(this->p_read_service_info(name, p_get_plugin_manifest(name)));
		RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "Create plugin: %s, base plugin: %s", name.c_str(), plugin->get_base_service_uri().c_str());
		std::shared_ptr<iop::PluginInterface> base_plugin = p_init_plugin(p_uri_to_name(plugin->get_base_service_uri()), class_loader);
		if (! base_plugin) {
			RCLCPP_INFO(p_rosnode->get_logger().get_child("PluginLoader"), "=== Initialize IOP-plugin for === %s ===", plugin->get_service_uri().c_str());
			plugin->create_service(this->jausRouter);
			JTS::Service* iop_service = plugin->get_service();
			RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "Initialized IOP-plugin for %s v%d.%d", plugin->get_service_uri().c_str(), plugin->get_version_number_major(), plugin->get_version_number_minor());
			ServiceInfo si(iop_service, plugin->get_service_uri());
			p_plugins_map[name] = plugin;
			service_list.push_back(si);
			if (plugin->is_discovery_client()) {
				p_discovery_client = plugin;
			}
		} else {
			std::vector<std::string> depends = plugin->get_depends();
			if (depends.size() > 0) {
				RCLCPP_INFO(p_rosnode->get_logger().get_child("PluginLoader"), "=== %s: base plugin '%s' found, check depend plugins...", name.c_str(), plugin->get_base_service_uri().c_str());
				for (unsigned int i = 0; i < depends.size(); i++) {
					std::shared_ptr<iop::PluginInterface> depplug = p_init_plugin(p_uri_to_name(depends[i]), class_loader);
					if (! depplug) {
						throw std::logic_error("required plugin " + depends[i] + " for service '" + name + "' not found");
					}
				}
			}
			RCLCPP_INFO(p_rosnode->get_logger().get_child("PluginLoader"), "=== Initialize IOP-plugin for === %s === <base service: %s>", plugin->get_service_uri().c_str(), base_plugin->get_service_uri().c_str());
			std::string suri = std::string(plugin->get_base_service_uri());
			if (suri.compare(base_plugin->get_service_uri()) == 0) {
				if (base_plugin->get_version_number_major() == plugin->get_base_version_manjor()
						&& plugin->get_base_min_version_minor() <= base_plugin->get_version_number_minor()) {
				} else {
					RCLCPP_WARN(p_rosnode->get_logger(), "%s has insufficient version %d.%d, required %d.%d", suri.c_str(),
							base_plugin->get_version_number_major(), base_plugin->get_version_number_minor(),
							plugin->get_version_number_major(), plugin->get_version_number_minor());
					throw std::logic_error("base service " + base_plugin->get_service_uri() + " for " + suri + " has insufficient version");
				}
			} else {
				throw std::logic_error("base plugin for service '" + plugin->get_service_uri() + "' not found");
			}
			plugin->set_base_plugin(base_plugin.get());
			plugin->create_service(this->jausRouter);
			JTS::Service* iop_service = plugin->get_service();
			RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "Initialization for %s v%d.%d done.", plugin->get_service_uri().c_str(), plugin->get_version_number_major(), plugin->get_version_number_minor());
			ServiceInfo si(iop_service, plugin->get_service_uri());
			p_plugins_map[name] = plugin;
			service_list.push_back(si);
			if (plugin->is_discovery_client()) {
				p_discovery_client = plugin;
			}
			plugin->error_message = "";
		}
		return plugin;
	}
	return std::shared_ptr<iop::PluginInterface>();
}

void Component::p_read_manifests(pluginlib::ClassLoader<iop::PluginInterface>& class_loader)
{

	RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "read manifests...");
	std::vector<std::string> classes = class_loader.getDeclaredClasses();
	for(unsigned int c = 0; c < classes.size(); ++c)
	{
		RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "  read manifest for %s", classes[c].c_str());
		try {
			std::shared_ptr<iop::PluginInterface> plugin = class_loader.createSharedInstance(classes[c]);
			plugin->set_service_info(this->p_read_service_info(classes[c], p_get_plugin_manifest(classes[c])));
			p_plugins_empty[classes[c]] = plugin;
		} catch (std::exception &e) {
			RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "%s", e.what());
		}
	}
	RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "read manifests done");
}

std::string Component::p_get_plugin_manifest(std::string plugin_name)
{
	std::string xml_path = "";
	std::map<std::string, std::string >::iterator it = p_service_package_list.find(plugin_name);
	if (it != p_service_package_list.end()) {
		std::string package = it->second;
		xml_path = p_class_loader->getPluginManifestPath(plugin_name);
		std::size_t found = xml_path.find(package);
		if (found != std::string::npos) {
			RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "  Assigned: [%s, %s] %s", plugin_name.c_str(), package.c_str(), xml_path.c_str());
		} else {
			RCLCPP_ERROR(p_rosnode->get_logger(), "No xml with service definition for '%s' in package '%s' found!", plugin_name.c_str(), p_service_package_list[plugin_name].c_str());
			throw std::logic_error("No xml with service definition for '" + plugin_name + "' in package '" + p_service_package_list[plugin_name] + "' found!");
		}
	} else {
		// plugin was not defined in our list, serch for a default one
		RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "service '%s' not found in parameter list, try to load from ROS package path...", plugin_name.c_str());
		xml_path = p_class_loader->getPluginManifestPath(plugin_name);
		RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "  Assigned: [%s] %s", plugin_name.c_str(), xml_path.c_str());
		if (xml_path.empty()) {
			RCLCPP_ERROR(p_rosnode->get_logger().get_child("PluginLoader"), "No xml with service definition for '%s' found!", plugin_name.c_str());
			throw std::logic_error("No xml with service definition for '" + plugin_name + "' found!");
		}
	}
	return xml_path;
}

std::string Component::p_uri_to_name(std::string service_uri)
{
	std::string result;
	std::map<std::string, std::shared_ptr<iop::PluginInterface> >::iterator it;
	// search in already created plugins
	for (it = p_plugins_map.begin(); it != p_plugins_map.end(); ++it) {
		if (it->second) {
			if (service_uri.compare(it->second->get_service_uri()) == 0) {
				return it->first;
			}
		}
	}
	// search in readed manifests
	for (it = p_plugins_empty.begin(); it != p_plugins_empty.end(); ++it) {
		if (it->second) {
			if (service_uri.compare(it->second->get_service_uri()) == 0) {
				return it->first;
			}
		}
	}
	return result;
}

bool Component::has_service(std::string service_uri)
{
	std::map<std::string, std::shared_ptr<iop::PluginInterface> >::iterator it;
	for (it = p_plugins_map.begin(); it != p_plugins_map.end(); ++it) {
		if (it->second.get()->get_service_uri().compare(service_uri) == 0) {
			return true;
		}
	}
	return false;
}

void Component::start_component()
{
	Service* service;

	jausRouter->start();
	this->start();

	RCLCPP_INFO(p_rosnode->get_logger(), "Start Services...");
	for (unsigned int i = 0; i < service_list.size(); i++)
	{
		RCLCPP_DEBUG(p_rosnode->get_logger(), "Start Service: %s", service_list.at(i).uri.c_str());
		service = service_list.at(i).service;
		service->start();
	}
	RCLCPP_INFO(p_rosnode->get_logger(), "Start Services...done!");

}


void Component::shutdown_component()
{
	Service* service;

	std::cout << "Shutdown component..." << std::endl;
	for (unsigned int i = 0; i < service_list.size(); i++)
	{
		std::cout << "  stop service: " << service_list.at(i).uri << " ..."<< std::endl;
		service = service_list.at(i).service;
		service->stop();
	}

	this->stop();
	jausRouter->stop();
}


JTS::Service* Component::get_service(std::string service_name)
{
	std::map<std::string, std::shared_ptr<iop::PluginInterface> >::iterator it;
	for (it = p_plugins_map.begin(); it != p_plugins_map.end(); ++it) {
		if (it->second.get()->get_service_name().compare(service_name) == 0
				&& it->second.get()->get_service() != NULL) {
			return it->second.get()->get_service();
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
	for (unsigned int i = service_list.size(); i>0; i--)
	{
		if (!done) {
			if (ie == 0) {
				RCLCPP_WARN(p_rosnode->get_logger(), "Error while cast JAUS message to right transport version, result is 0");
			} else {
				done = service_list.at(i-1).service->processTransitions(ie);
				if (done) {
					Receive* casted_ie = (Receive*) ie;
					unsigned short id = *((unsigned short*) casted_ie->getBody()->getReceiveRec()->getMessagePayload()->getData());
					RCLCPP_DEBUG(p_rosnode->get_logger().get_child("InternalProcess"), "PROCESSED: %s - message type: %x, transition: %s", service_list.at(i-1).uri.c_str(), id, casted_ie->getName().c_str());
				}
			}
		}
	}
	for (unsigned int i = service_list.size(); i>0; i--)
	{
		if (!done) {
			if (ie == 0) {
				RCLCPP_WARN(p_rosnode->get_logger(), "Error while cast JAUS message to right transport version, result is 0");
			} else {
				done = service_list.at(i-1).service->defaultTransitions(ie);
				if (done) {
					Receive* casted_ie = (Receive*) ie;
					unsigned short id = *((unsigned short*) casted_ie->getBody()->getReceiveRec()->getMessagePayload()->getData());
					RCLCPP_DEBUG(p_rosnode->get_logger().get_child("InternalProcess"), "PROCESSED DEFAULT: %s - message type: %x, transition: %s", service_list.at(i-1).uri.c_str(), id, casted_ie->getName().c_str());
				}
			}
		}
	}
	if (!done) {
		Receive* casted_ie = (Receive*) ie;
		unsigned short id = *((unsigned short*) casted_ie->getBody()->getReceiveRec()->getMessagePayload()->getData());
		RCLCPP_DEBUG(p_rosnode->get_logger().get_child("InternalProcess"), "NOT PROCESSED: %x, transition: %s, source: %s", id, ie->getName().c_str(), ie->getSource().c_str());
	}
}

iop::PluginInterface::ServiceInfo Component::p_read_service_info(std::string serviceid, std::string manifest)
{
	iop::PluginInterface::ServiceInfo result;
	// lookup in cache
	if ( p_cache_service_info.find(serviceid) != p_cache_service_info.end() )
	{
		return p_cache_service_info[serviceid];
	}

	if (!manifest.empty())
	{
		RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "Parsing %s", manifest.c_str());
		tinyxml2::XMLDocument document;
		document.LoadFile(manifest.c_str());
		tinyxml2::XMLElement* config = document.RootElement();
		if (config == NULL)
		{
			RCLCPP_ERROR(p_rosnode->get_logger().get_child("PluginLoader"), "Skipping XML Document \"%s\" which had no Root Element.  This likely means the XML is malformed or missing.", manifest.c_str());
			throw std::logic_error("Wrong iop plugin definition!");
		}
		if (strcmp(config->Value(), "library") != 0 &&
				strcmp(config->Value(), "class_libraries") != 0)
		{
			RCLCPP_ERROR(p_rosnode->get_logger().get_child("PluginLoader"), "The XML document \"%s\" given to add must have either \"library\" or \"class_libraries\" as the root tag", manifest.c_str());
			throw std::logic_error("Wrong iop plugin definition!");
		}
		//Step into the filter list if necessary
		if (strcmp(config->Value(), "class_libraries") == 0)
		{
			config = config->FirstChildElement("library");
		}

		tinyxml2::XMLElement* library = config;
		while ( library != NULL)
		{
			tinyxml2::XMLElement* class_element = library->FirstChildElement("class");
			while (class_element)
			{
				// test for class name and skip parsing if it is wrong class id
				std::string classid("");
				if (class_element->Attribute("name") != NULL)
				{
					classid = class_element->Attribute("name");
					RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file specifies class name = %s", classid.c_str());
					if (classid.compare(serviceid) != 0) {
						RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "  %s is not searching service, skip", classid.c_str());
						//step to next class_element
						class_element = class_element->NextSiblingElement( "class" );
						continue;
					}
				} else if (class_element->Attribute("type") != NULL)
				{
					classid = class_element->Attribute("type");
					RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file specifies class type = %s", classid.c_str());
					if (classid.compare(serviceid) != 0) {
						RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "  %s is not searching service, skip", classid.c_str());
						//step to next class_element
						class_element = class_element->NextSiblingElement( "class" );
						continue;
					}
				}

				// format: <iop_service name="Events" id="urn:jaus:jss:core:Events" version="1.0">
				tinyxml2::XMLElement* service_element = class_element->FirstChildElement("iop_service");
				if (service_element) {
					result.id = service_element->Attribute("id");
					if (service_element->Attribute("name") != NULL)
					{
						result.name = service_element->Attribute("name");
						RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file specifies type = %s", result.name.c_str());
					} else {
						RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file has no type for service id %s, let it empty.", result.id.c_str());
					}
					// get version
					result.version_manjor = 1;
					result.version_minor = 1;
					if (service_element->Attribute("version") != NULL) {
						std::string version = service_element->Attribute("version");
						int p1, p2;
						int scan_result = std::sscanf(version.c_str(), "%d.%d", &p1, &p2);
						if (scan_result == 2) {
							result.version_manjor = p1;
							result.version_minor = p2;
							RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file specifies version = %d.%d", p1, p2);
						} else if (scan_result == 1) {
							result.version_manjor = p1;
							RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file specifies only major version = %d", p1);
						} else {
							RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file has wrong version attribute value = %s", version.c_str());
						}
					} else {
						RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file has no version for service id %s, assume 1.0", result.id.c_str());
					}
					// get required_service spec, format: <inherits_from id="urn:jaus:jss:core:Transport" min_version="1.0"/>
					result.inherits_from = "";
					tinyxml2::XMLElement* inherits_from = service_element->FirstChildElement("inherits_from");
					if (inherits_from) {
						if (inherits_from->Attribute("id") != NULL)
						{
							result.inherits_from = inherits_from->Attribute("id");
							RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file specifies inherits_from type = %s", result.inherits_from.c_str());
						} else {
							RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file has no id in inherits_from tag.");
						}
						// get version
						result.inherits_from_version_manjor = 1;
						result.inherits_from_min_version_minor = 1;
						if (inherits_from->Attribute("min_version") != NULL) {
							std::string version = inherits_from->Attribute("min_version");
							int p1, p2;
							int scan_result = std::sscanf(version.c_str(), "%d.%d", &p1, &p2);
							if (scan_result == 2) {
								result.inherits_from_version_manjor = p1;
								result.inherits_from_min_version_minor = p2;
								RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file specifies version = %d.%d", p1, p2);
							} else if (scan_result == 1) {
								result.inherits_from_version_manjor = p1;
								RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file specifies only major min version = %d", p1);
							} else {
								RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file has for inherits_from wrong version attribute value = %s", version.c_str());
							}
						} else {
							RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file has no min version for inherits_from id %s, assume 1.0", result.inherits_from.c_str());
						}
					}
					// get plugins depend on, format: <depend id="urn:jaus:jss:core:Transport"/>
					tinyxml2::XMLElement* depend_on = service_element->FirstChildElement("depend");
					while (depend_on) {
						if (depend_on->Attribute("id") != NULL)
						{
							std::string depend_id = depend_on->Attribute("id");
							result.depend.push_back(depend_id);
							RCLCPP_DEBUG(p_rosnode->get_logger().get_child("PluginLoader"), "XML file specifies depend service type = %s", depend_id.c_str());
						} else {
							RCLCPP_WARN(p_rosnode->get_logger().get_child("PluginLoader"), "XML file has no id in depend tag: %s", manifest.c_str());
						}
						depend_on = depend_on->NextSiblingElement("depend");
					}
					p_cache_service_info[serviceid] = result;
				} else {
					RCLCPP_ERROR(p_rosnode->get_logger().get_child("PluginLoader"), "iop_service definition in plugin definition %s not found!", manifest.c_str());
					throw std::logic_error("Wrong iop plugin definition!");
				}
				//step to next class_element
				class_element = class_element->NextSiblingElement( "class" );
			}
			library = library->NextSiblingElement( "library" );
		}
	}

	if ( p_cache_service_info.find(serviceid) != p_cache_service_info.end() )
	{
		return p_cache_service_info[serviceid];
	}
	return iop::PluginInterface::ServiceInfo();
}
