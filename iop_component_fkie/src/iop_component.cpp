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
			// TODO: try to detect configuration path from jaustoolset package
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
	std::vector<boost::shared_ptr<iop::PluginInterface> >::iterator it;
	for (it = p_plugins.begin(); it != p_plugins.end(); ++it) {
		std::cout << "  delete service:" << it->get()->get_service_uri() << std::endl;
		delete it->get()->get_service();
	}
	p_plugins.clear();
	service_list.clear();
	delete p_class_loader;
	delete jausRouter;
	std::cout << "Shutdown component finished" << std::endl;
}

void Component::load_plugins()
{
	bool has_1_1_transport = false;
	boost::shared_ptr<iop::PluginInterface> discovery_client;
	std::map<std::string, std::string > service_package_list;
	std::vector<std::string> plugin_names;
	XmlRpc::XmlRpcValue v;
	p_pnh.getParam("services", v);
	if (!v.valid()) {
		p_pnh.getParam("services", v);
	}
	if (!v.valid()) {
		throw std::logic_error("~service parameter seems to be invalid!");
	}
	ROS_INFO("Load IOP plugin services specified by ~services parameter:");
	for(int i = 0; i < v.size(); i++) {
		if (v[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
			for(XmlRpc::XmlRpcValue::ValueStruct::iterator iterator = v[i].begin(); iterator != v[i].end(); iterator++) {
				std::string package = iterator->first;
				std::string service = iterator->second;
				service_package_list[service] = package;
				plugin_names.push_back(service);
			}
		}
	}
	// determine paths for xml files with plugin description
	std::vector<std::string> p_xml_paths;
	p_class_loader = new pluginlib::ClassLoader<iop::PluginInterface>("iop_component_fkie", "iop::PluginInterface", std::string("plugin"), p_xml_paths);
	std::vector<std::string> classes = p_class_loader->getDeclaredClasses();
	std::map< std::pair<std::string, std::string>, std::string> mpaths;  // <pair<service, package>, path_to_xml>
	for (unsigned int i = 0; i < plugin_names.size(); ++i)
	{
		bool found_xml = false;
		for(unsigned int c = 0; c < classes.size(); ++c)
		{
			if (classes[c].compare(plugin_names[i]) == 0)
			{
				std::string package = service_package_list[classes[c]];
				std::string xml_path = p_class_loader->getPluginManifestPath(classes[c].c_str());
				std::size_t found = xml_path.find(package);
				if (found != std::string::npos)
				{
					found_xml = true;
					mpaths[std::make_pair(classes[c], package)] = xml_path;
					ROS_DEBUG("  Assigned: [%s, %s], path to manifest: %s", classes[c].c_str(), package.c_str(), xml_path.c_str());
				}
			}
		}
		if (!found_xml)
		{
			ROS_ERROR("No xml with service definition for '%s' in package '%s' found!", plugin_names[i].c_str(), service_package_list[plugin_names[i]].c_str());
			throw std::logic_error("no manifest found");
		}
	}
	try
	{
		for (unsigned int i = 0; i < plugin_names.size(); ++i)
		{
			// initialize all services without defined base services
			std::pair<std::string, std::string> svr_pkg_pair = std::make_pair(plugin_names[i], service_package_list[plugin_names[i]]);
			boost::shared_ptr<iop::PluginInterface> plugin = p_class_loader->createInstance(plugin_names[i]);
			plugin->set_service_info(this->p_read_service_info(plugin_names[i], mpaths[svr_pkg_pair]));
			if (plugin->get_base_service_uri().empty()) {
				ROS_INFO("Initialize IOP-plugin for %s", plugin->get_service_uri().c_str());
				plugin->create_service(this->jausRouter);
				JTS::Service* iop_service = plugin->get_service();
				ROS_DEBUG("Initialized IOP-plugin for %s v%d.%d", plugin->get_service_uri().c_str(), plugin->get_version_number_major(), plugin->get_version_number_minor());
				bool is_transport_1_1 = jausRouter->getTransportType() == JTS::JausRouter::Version_1_1;
				has_1_1_transport = has_1_1_transport | is_transport_1_1;
				ServiceInfo si(iop_service, plugin->get_service_uri(), is_transport_1_1);
				service_list.push_back(si);
				if (plugin->is_discovery_client()) {
					discovery_client = plugin;
				}
			}
			p_plugins.push_back(plugin);
		}
		bool one_initialized;
		do
		{
			// initialize all plugins with defined base services
			one_initialized = false;
			for (unsigned int i = 0; i < p_plugins.size(); ++i)
			{
				if (p_plugins[i]->get_service() == NULL) {
					try
					{
						boost::shared_ptr<iop::PluginInterface> base_plugin = p_get_plugin_str(p_plugins[i]->get_base_service_uri(), p_plugins[i]->get_base_version_manjor(), p_plugins[i]->get_base_min_version_minor());
						if (base_plugin != NULL) {
							ROS_INFO("Initialize IOP-plugin for %s <base service: %s>", p_plugins[i]->get_service_uri().c_str(), base_plugin->get_service_uri().c_str());
							p_plugins[i]->set_base_plugin(base_plugin.get());
							p_plugins[i]->create_service(this->jausRouter);
							JTS::Service* iop_service = p_plugins[i]->get_service();
							ROS_DEBUG("Initialization for %s v%d.%d done.", p_plugins[i]->get_service_uri().c_str(), p_plugins[i]->get_version_number_major(), p_plugins[i]->get_version_number_minor());
							bool is_transport_1_1 = jausRouter->getTransportType() == JTS::JausRouter::Version_1_1;
							has_1_1_transport = has_1_1_transport | is_transport_1_1;
							ServiceInfo si(iop_service, p_plugins[i]->get_service_uri(), is_transport_1_1);
							service_list.push_back(si);
							one_initialized = true;
							if (p_plugins[i]->is_discovery_client()) {
								discovery_client = p_plugins[i];
							}
						}
					} catch (std::runtime_error &rex){
						throw rex;
					} catch (std::logic_error &no_plugin){
						// The needed plugin is still not available, perhaps in the next iteration.
						// The component stops if one_initialized was not set to true.
					}
				}
			}
		} while (one_initialized);
	}
	catch(pluginlib::PluginlibException& ex)
	{
		throw std::runtime_error("The plugin failed to load for some reason. Error: " + std::string(ex.what()));
	}
	// test for uninitialized pugins
	for (unsigned int i = 0; i < p_plugins.size(); ++i)
	{
		if (p_plugins[i]->get_service() == NULL) {
			throw std::runtime_error("Can not initialize plugin for " +
					std::string(p_plugins[i]->get_service_name().c_str()) +
					". Plugin for base service " +
					std::string(p_plugins[i]->get_base_service_uri().c_str()) + " not found!");
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
		p_plugins[i]->init_service();
	}
	if (has_1_1_transport) {
		ROS_INFO("Use transport version 1.1");
		jausRouter->setTransportType(JausRouter::Version_1_1);
	} else {
		ROS_INFO("Use transport version 1.0");
	}
	ROS_INFO("... initialization complete");
}


boost::shared_ptr<iop::PluginInterface> Component::p_get_plugin_str(const std::string service_uri, unsigned char major_version, unsigned char min_minor_version)
{
	for (unsigned int i = 0; i < p_plugins.size(); ++i)
	{
		std::string suri = std::string(p_plugins[i]->get_service_uri());
		if (suri.compare(service_uri) == 0
				&& p_plugins[i]->get_service() != NULL) {
			if (major_version == p_plugins[i]->get_version_number_major()
					&& min_minor_version <= p_plugins[i]->get_version_number_minor()){
				return p_plugins[i];
			} else {
				ROS_WARN("%s has insufficient version %d.%d, required %d.%d", suri.c_str(), p_plugins[i]->get_version_number_major(), p_plugins[i]->get_version_number_minor(), major_version, min_minor_version);
			}
		}
	}
	throw std::logic_error("plugin is not available");
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
	for (unsigned int i = 0; i < p_plugins.size(); ++i)
	{
		if (p_plugins[i]->get_service_name().compare(service_name) == 0
				&& p_plugins[i]->get_service() != NULL) {
//			std::cout << "get service '" << service_name << "', uri: " << p_plugins[i]->get_service_uri()  << ", type: " << typeid(p_plugins[i]->get_service()).name() << std::endl;
			return p_plugins[i]->get_service();
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
					ROS_DEBUG_NAMED("InternalProcess", "PROCESSED: %s - transport_type: %d, message type: %x", service_list.at(i-1).uri.c_str(), service_list.at(i-1).transport_type, id);
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
					ROS_DEBUG_NAMED("InternalProcess", "PROCESSED DEFAULT: %s - transport_type: %d, message type: %x", service_list.at(i-1).uri.c_str(), service_list.at(i-1).transport_type, id);
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
							ROS_DEBUG("XML file specifies required service type = %s", result.inherits_from.c_str());
						} else {
							ROS_DEBUG("XML file has no type in required_service tag.");
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
								ROS_DEBUG("XML file has for required service wrong version attribute value = %s", version.c_str());
							}
						} else {
							ROS_DEBUG("XML file has no min version for required service id %s, assume 1.0", result.inherits_from.c_str());
						}
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
