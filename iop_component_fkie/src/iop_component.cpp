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
#include <tinyxml.h>
#include "iop_component_fkie/iop_component.h"
#include "JausUtils.h"
#include <XmlRpcException.h>

using namespace JTS;
using namespace iop;

Component* Component::global_ptr = 0;

Component::Component(unsigned int subsystem, unsigned short node, unsigned short component)
{
	global_ptr = this;
	p_pnh = ros::NodeHandle("~");
	std::string config("nm.cfg");
	if (p_pnh.hasParam("jaus_config")) {
		// read configuration from private parameter
		p_pnh.getParam("jaus_config", config);
	} else {
		ros::NodeHandle nh;
		if (nh.hasParam("jaus_config")) {
			// read configuration from namespace parameter
			nh.getParam("jaus_config", config);
		} else {
			// try to detect from jaustoolset package
			config = ros::package::getPath("jaustoolset") + "/cfg/nm.cfg";
		}
	}
	if (config.compare("nm.cfg") == 0) {
		char cwd[1024];
		if (getcwd(cwd, sizeof(cwd)) != NULL) {
			ROS_INFO("JAUS configuration file: %s, pwd: %s", config.c_str(), cwd);
		} else {
			ROS_INFO("JAUS configuration file: %s", config.c_str());
		}
	} else {
		ROS_INFO("JAUS configuration file: %s", config.c_str());
	}
	this->jausRouter = new IopJausRouter(JausAddress(subsystem, node, component), ieHandler, config);
	while (ros::ok() && ! jausRouter->isConnected()) {
		ROS_INFO("Try reconnect to JAUS nodeManager...");
		sleep(1);
		delete this->jausRouter;
		this->jausRouter = new IopJausRouter(JausAddress(subsystem, node, component), ieHandler, config);
	}
	if (ros::ok() && jausRouter->isConnected()) {
		p_class_loader = NULL;
		ROS_INFO("JAUS ID: %d", this->jausRouter->getJausAddress()->get());
		load_plugins();
	}
}

Component::~Component()
{
	std::map<std::string, boost::shared_ptr<iop::PluginInterface> >::iterator it;
	for (it = p_plugins_map.begin(); it != p_plugins_map.end(); ++it) {
		std::cout << "  delete service:" << it->second.get()->get_service_uri() << std::endl;
		delete it->second.get()->get_service();
	}
	p_plugins_map.clear();
	service_list.clear();
	delete p_class_loader;
	delete jausRouter;
	std::cout << "Shutdown component finished" << std::endl;
}

void Component::load_plugins()
{
	p_has_1_1_transport = false;
	p_discovery_client = boost::shared_ptr<iop::PluginInterface>();
	std::vector<std::string> plugin_names;
	XmlRpc::XmlRpcValue v;
	p_pnh.getParam("services", v);
	if (!v.valid()) {
		p_pnh.getParam("services", v);
	}
	if (!v.valid()) {
		throw std::logic_error("~service parameter seems to be invalid!");
	}
	bool has_range_sensor_service = false;
	bool has_visual_sensor_service = false;
	bool has_discovery_service = false;
	bool has_discovery_client_service = false;
	ROS_INFO("Load IOP plugin services specified by ~services parameter:");
	for(int i = 0; i < v.size(); i++) {
		if (v[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
			for(XmlRpc::XmlRpcValue::ValueStruct::iterator iterator = v[i].begin(); iterator != v[i].end(); iterator++) {
				std::string package = iterator->first;
				std::string service = iterator->second;
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
			}
		}
	}
	if (has_range_sensor_service & has_discovery_client_service) {
		ROS_WARN("In this version you do not need to include Discovery and DiscoverClient in the same component!");
	}
	if (has_discovery_service & has_visual_sensor_service) {
		throw std::logic_error("You can not use RangeSensor{Client} and VisualSensor{Client} in the same component, since they use the same GeometricProperties message type!");
	}
	// determine paths for xml files with plugin description
	std::vector<std::string> p_xml_paths;
	p_class_loader = new pluginlib::ClassLoader<iop::PluginInterface>("iop_component_fkie", "iop::PluginInterface", std::string("plugin"), p_xml_paths);
	p_read_manifests(*p_class_loader);
	try {
		for (unsigned int i = 0; i < plugin_names.size(); ++i) {
			p_init_plugin(plugin_names[i], *p_class_loader);
		}
	} catch(pluginlib::PluginlibException& ex) {
		throw std::runtime_error("The plugin failed to load for some reason. Error: " + std::string(ex.what()));
	}
	// test for uninitialized pugins
	std::map<std::string, boost::shared_ptr<iop::PluginInterface> >::iterator it;
	for (it = p_plugins_map.begin(); it != p_plugins_map.end(); ++it) {
		if (it->second.get()->get_service() == NULL) {
			throw std::runtime_error(it->second.get()->error_message);
		}
	}
	ROS_INFO("... plugin loading complete!");
	// register the services
	if (p_discovery_client != NULL) {
		ROS_INFO("Register services by discovery service...");
		for (it = p_plugins_map.begin(); it != p_plugins_map.end(); ++it) {
			p_discovery_client->register_service(it->second.get());
		}
		ROS_INFO("... register complete");
	} else {
		ROS_INFO("Discovery client not found, the services will not be registered!");
	}
	ROS_INFO("Initialize plugins ...");
	for (it = p_plugins_map.begin(); it != p_plugins_map.end(); ++it) {
		it->second.get()->init_service();
	}
	if (p_has_1_1_transport) {
		ROS_INFO("Use transport version 1.1");
		jausRouter->setTransportType(JausRouter::Version_1_1);
	} else {
		ROS_INFO("Use transport version 1.0");
	}
	ROS_INFO("... initialization complete");
}

boost::shared_ptr<iop::PluginInterface> Component::p_init_plugin(std::string name, pluginlib::ClassLoader<iop::PluginInterface>& class_loader)
{
	if (name.empty()) {
		return boost::shared_ptr<iop::PluginInterface>();
	}
	std::map<std::string, boost::shared_ptr<iop::PluginInterface> >::iterator it = p_plugins_map.find(name);
	if (it != p_plugins_map.end()) {
		return it->second;
	} else {
		boost::shared_ptr<iop::PluginInterface> plugin = class_loader.createInstance(name);
		plugin->set_service_info(this->p_read_service_info(name, p_get_plugin_manifest(name)));
		ROS_DEBUG_NAMED("PluginLoader", "Create plugin: %s, base plugin: %s", name.c_str(), plugin->get_base_service_uri().c_str());
		boost::shared_ptr<iop::PluginInterface> base_plugin = p_init_plugin(p_uri_to_name(plugin->get_base_service_uri()), class_loader);
		if (! base_plugin) {
			ROS_INFO_NAMED("PluginLoader", "=== Initialize IOP-plugin for === %s ===", plugin->get_service_uri().c_str());
			plugin->create_service(this->jausRouter);
			JTS::Service* iop_service = plugin->get_service();
			ROS_DEBUG_NAMED("PluginLoader", "Initialized IOP-plugin for %s v%d.%d", plugin->get_service_uri().c_str(), plugin->get_version_number_major(), plugin->get_version_number_minor());
			bool is_transport_1_1 = jausRouter->getTransportType() == JTS::JausRouter::Version_1_1;
			p_has_1_1_transport = p_has_1_1_transport | is_transport_1_1;
			ServiceInfo si(iop_service, plugin->get_service_uri(), is_transport_1_1);
			p_plugins_map[name] = plugin;
			service_list.push_back(si);
			if (plugin->is_discovery_client()) {
				p_discovery_client = plugin;
			}
		} else {
			std::vector<std::string> depends = plugin->get_depends();
			if (depends.size() > 0) {
				ROS_INFO_NAMED("PluginLoader", "=== %s: base plugin '%s' found, check depend plugins...", name.c_str(), plugin->get_base_service_uri().c_str());
				for (unsigned int i = 0; i < depends.size(); i++) {
					boost::shared_ptr<iop::PluginInterface> depplug = p_init_plugin(p_uri_to_name(depends[i]), class_loader);
					if (! depplug) {
						throw std::logic_error("required plugin " + depends[i] + " for service '" + name + "' not found");
					}
				}
			}
			ROS_INFO_NAMED("PluginLoader", "=== Initialize IOP-plugin for === %s === <base service: %s>", plugin->get_service_uri().c_str(), base_plugin->get_service_uri().c_str());
			std::string suri = std::string(plugin->get_base_service_uri());
			if (suri.compare(base_plugin->get_service_uri()) == 0) {
				if (base_plugin->get_version_number_major() == plugin->get_base_version_manjor()
						&& plugin->get_base_min_version_minor() <= base_plugin->get_version_number_minor()) {
				} else {
					ROS_WARN("%s has insufficient version %d.%d, required %d.%d", suri.c_str(),
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
			ROS_DEBUG_NAMED("PluginLoader", "Initialization for %s v%d.%d done.", plugin->get_service_uri().c_str(), plugin->get_version_number_major(), plugin->get_version_number_minor());
			bool is_transport_1_1 = jausRouter->getTransportType() == JTS::JausRouter::Version_1_1;
			p_has_1_1_transport = p_has_1_1_transport | is_transport_1_1;
			ServiceInfo si(iop_service, plugin->get_service_uri(), is_transport_1_1);
			p_plugins_map[name] = plugin;
			service_list.push_back(si);
			if (plugin->is_discovery_client()) {
				p_discovery_client = plugin;
			}
			plugin->error_message = "";
		}
		return plugin;
	}
	return boost::shared_ptr<iop::PluginInterface>();
}

void Component::p_read_manifests(pluginlib::ClassLoader<iop::PluginInterface>& class_loader)
{

	ROS_DEBUG_NAMED("PluginLoader", "read manifests...");
	std::vector<std::string> classes = class_loader.getDeclaredClasses();
	for(unsigned int c = 0; c < classes.size(); ++c)
	{
		ROS_DEBUG_NAMED("PluginLoader", "  read manifest for %s", classes[c].c_str());
		try {
			boost::shared_ptr<iop::PluginInterface> plugin = class_loader.createInstance(classes[c]);
			plugin->set_service_info(this->p_read_service_info(classes[c], p_get_plugin_manifest(classes[c])));
			p_plugins_empty[classes[c]] = plugin;
		} catch (std::exception &e) {
			ROS_DEBUG_NAMED("PluginLoader", "%s", e.what());
		}
	}
	ROS_DEBUG_NAMED("PluginLoader", "read manifests done");
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
			ROS_DEBUG_NAMED("PluginLoader", "  Assigned: [%s, %s] %s", plugin_name.c_str(), package.c_str(), xml_path.c_str());
		} else {
			ROS_ERROR("No xml with service definition for '%s' in package '%s' found!", plugin_name.c_str(), p_service_package_list[plugin_name].c_str());
			throw std::logic_error("No xml with service definition for '" + plugin_name + "' in package '" + p_service_package_list[plugin_name] + "' found!");
		}
	} else {
		// plugin was not defined in our list, serch for a default one
		ROS_DEBUG_NAMED("PluginLoader", "service '%s' not found in parameter list, try to load from ROS package path...", plugin_name.c_str());
		xml_path = p_class_loader->getPluginManifestPath(plugin_name);
		ROS_DEBUG_NAMED("PluginLoader", "  Assigned: [%s] %s", plugin_name.c_str(), xml_path.c_str());
		if (xml_path.empty()) {
			ROS_ERROR("No xml with service definition for '%s' found!", plugin_name.c_str());
			throw std::logic_error("No xml with service definition for '" + plugin_name + "' found!");
		}
	}
	return xml_path;
}

std::string Component::p_uri_to_name(std::string service_uri)
{
	std::string result;
	std::map<std::string, boost::shared_ptr<iop::PluginInterface> >::iterator it;
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
	std::map<std::string, boost::shared_ptr<iop::PluginInterface> >::iterator it;
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

	ROS_INFO("Start Services...");
	for (unsigned int i = 0; i < service_list.size(); i++)
	{
		ROS_DEBUG("Start Service: %s", service_list.at(i).uri.c_str());
		service = service_list.at(i).service;
		service->start();
	}
	ROS_INFO("Start Services...done!");

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
	std::map<std::string, boost::shared_ptr<iop::PluginInterface> >::iterator it;
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
				ROS_WARN("Error while cast JAUS message to right transport version, result is 0");
			} else {
				done = service_list.at(i-1).service->processTransitions(ie);
				if (done) {
					Receive* casted_ie = (Receive*) ie;
					unsigned short id = *((unsigned short*) casted_ie->getBody()->getReceiveRec()->getMessagePayload()->getData());
					ROS_DEBUG_NAMED("InternalProcess", "PROCESSED: %s - transport_type: %d, message type: %x, transition: %s", service_list.at(i-1).uri.c_str(), service_list.at(i-1).transport_type, id, casted_ie->getName().c_str());
				}
			}
		}
	}
	for (unsigned int i = service_list.size(); i>0; i--)
	{
		if (!done) {
			if (ie == 0) {
				ROS_WARN("Error while cast JAUS message to right transport version, result is 0");
			} else {
				done = service_list.at(i-1).service->defaultTransitions(ie);
				if (done) {
					Receive* casted_ie = (Receive*) ie;
					unsigned short id = *((unsigned short*) casted_ie->getBody()->getReceiveRec()->getMessagePayload()->getData());
					ROS_DEBUG_NAMED("InternalProcess", "PROCESSED DEFAULT: %s - transport_type: %d, message type: %x, transition: %s", service_list.at(i-1).uri.c_str(), service_list.at(i-1).transport_type, id, casted_ie->getName().c_str());
				}
			}
		}
	}
	if (!done) {
		Receive* casted_ie = (Receive*) ie;
		unsigned short id = *((unsigned short*) casted_ie->getBody()->getReceiveRec()->getMessagePayload()->getData());
		ROS_DEBUG_NAMED("InternalProcess", "NOT PROCESSED: %x, transition: %s, source: %s", id, ie->getName().c_str(), ie->getSource().c_str());
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
		ROS_DEBUG_STREAM("Parsing " << manifest);
		TiXmlDocument document;
		document.LoadFile(manifest);
		TiXmlElement * config = document.RootElement();
		if (config == NULL)
		{
			ROS_ERROR("Skipping XML Document \"%s\" which had no Root Element.  This likely means the XML is malformed or missing.", manifest.c_str());
			throw std::logic_error("Wrong iop plugin definition!");
		}
		if (config->ValueStr() != "library" &&
				config->ValueStr() != "class_libraries")
		{
			ROS_ERROR("The XML document \"%s\" given to add must have either \"library\" or \"class_libraries\" as the root tag", manifest.c_str());
			throw std::logic_error("Wrong iop plugin definition!");
		}
		//Step into the filter list if necessary
		if (config->ValueStr() == "class_libraries")
		{
			config = config->FirstChildElement("library");
		}

		TiXmlElement* library = config;
		while ( library != NULL)
		{
			TiXmlElement* class_element = library->FirstChildElement("class");
			while (class_element)
			{
				// test for class name and skip parsing if it is wrong class id
				std::string classid("");
				if (class_element->Attribute("name") != NULL)
				{
					classid = class_element->Attribute("name");
					ROS_DEBUG("XML file specifies class name = %s", classid.c_str());
					if (classid.compare(serviceid) != 0) {
						ROS_DEBUG("  %s is not searching service, skip", classid.c_str());
						//step to next class_element
						class_element = class_element->NextSiblingElement( "class" );
						continue;
					}
				} else if (class_element->Attribute("type") != NULL)
				{
					classid = class_element->Attribute("type");
					ROS_DEBUG("XML file specifies class type = %s", classid.c_str());
					if (classid.compare(serviceid) != 0) {
						ROS_DEBUG("  %s is not searching service, skip", classid.c_str());
						//step to next class_element
						class_element = class_element->NextSiblingElement( "class" );
						continue;
					}
				}

				// format: <iop_service name="Events" id="urn:jaus:jss:core:Events" version="1.0">
				TiXmlElement* service_element = class_element->FirstChildElement("iop_service");
				if (service_element) {
					result.id = service_element->Attribute("id");
					if (service_element->Attribute("name") != NULL)
					{
						result.name = service_element->Attribute("name");
						ROS_DEBUG("XML file specifies type = %s", result.name.c_str());
					} else {
						ROS_DEBUG("XML file has no type for service id %s, let it empty.", result.id.c_str());
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
							ROS_DEBUG("XML file specifies version = %d.%d", p1, p2);
						} else if (scan_result == 1) {
							result.version_manjor = p1;
							ROS_DEBUG("XML file specifies only major version = %d", p1);
						} else {
							ROS_DEBUG("XML file has wrong version attribute value = %s", version.c_str());
						}
					} else {
						ROS_DEBUG("XML file has no version for service id %s, assume 1.0", result.id.c_str());
					}
					// get required_service spec, format: <inherits_from id="urn:jaus:jss:core:Transport" min_version="1.0"/>
					result.inherits_from = "";
					TiXmlElement* inherits_from = service_element->FirstChildElement("inherits_from");
					if (inherits_from) {
						if (inherits_from->Attribute("id") != NULL)
						{
							result.inherits_from = inherits_from->Attribute("id");
							ROS_DEBUG("XML file specifies inherits_from type = %s", result.inherits_from.c_str());
						} else {
							ROS_DEBUG("XML file has no id in inherits_from tag.");
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
								ROS_DEBUG("XML file specifies version = %d.%d", p1, p2);
							} else if (scan_result == 1) {
								result.inherits_from_version_manjor = p1;
								ROS_DEBUG("XML file specifies only major min version = %d", p1);
							} else {
								ROS_DEBUG("XML file has for inherits_from wrong version attribute value = %s", version.c_str());
							}
						} else {
							ROS_DEBUG("XML file has no min version for inherits_from id %s, assume 1.0", result.inherits_from.c_str());
						}
					}
					// get plugins depend on, format: <depend id="urn:jaus:jss:core:Transport"/>
					TiXmlElement* depend_on = service_element->FirstChildElement("depend");
					while (depend_on) {
						if (depend_on->Attribute("id") != NULL)
						{
							std::string depend_id = depend_on->Attribute("id");
							result.depend.push_back(depend_id);
							ROS_DEBUG("XML file specifies depend service type = %s", depend_id.c_str());
						} else {
							ROS_DEBUG("XML file has no id in depend tag.");
						}
						depend_on = depend_on->NextSiblingElement("depend");
					}
					p_cache_service_info[serviceid] = result;
				} else {
					ROS_ERROR("iop_service definition in plugin definition %s not found!", manifest.c_str());
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
