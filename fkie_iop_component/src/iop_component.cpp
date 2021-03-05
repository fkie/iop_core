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
#include "fkie_iop_component/iop_component.hpp"
#include "fkie_iop_component/iop_config.hpp"
#include "fkie_iop_component/string.hpp"
#include "JausUtils.h"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

using namespace JTS;
using namespace iop;


Component::Component(const std::string node_name, const std::string namespace_):
	rclcpp::Node(node_name, namespace_,
		rclcpp::NodeOptions()
			.allow_undeclared_parameters(false)
			.automatically_declare_parameters_from_overrides(false)),
	JTS::EventReceiver()
{
	p_discovery_client = std::shared_ptr<JTS::Service>();
	p_cfg = nullptr;
	p_connect_thread = nullptr;
	p_id_subsystem = 0;
	p_id_node = 0;
	p_id_component = 0;
	p_search_for_id_params = true;
	p_iop_initialized = false;
	p_use_remote_time = false;
}

void Component::init(unsigned int subsystem, unsigned short node, unsigned short component)
{
	p_cfg = new iop::Config(std::dynamic_pointer_cast<iop::Component>(shared_from_this()), "");
	p_cfg->declare_param<std::string>("iop_address", "", true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"Address of the IOP component", "{subsystem-65535}.{node-255}.{component-255}");
	p_id_subsystem = subsystem;
	p_id_node = node;
	p_id_component = component;
	rclcpp::Parameter addr_param = this->get_parameter("iop_address");
	if (addr_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
		std::string addr_str = addr_param.as_string();
		if (!addr_str.empty()) {
			int p1, p2, p3;
			int scan_result = std::sscanf(addr_str.c_str(), "%d.%d.%d", &p1, &p2, &p3);
			if (scan_result == 2) {
				RCLCPP_INFO(this->get_logger(), "found iop_address: %s", addr_str.c_str());
				p_id_subsystem = p1;
				p_id_node = p2;
				p_search_for_id_params = false;
			} else if (scan_result == 3) {
				RCLCPP_INFO(this->get_logger(), "found iop_address: %s", addr_str.c_str());
				p_id_subsystem = p1;
				p_id_node = p2;
				p_id_component = p3;
				p_search_for_id_params = false;
			} else {
				RCLCPP_WARN(this->get_logger(), "invalid format in iop_address[str]: %s, should be subsystem.node.component or subsystem.node", addr_str.c_str());
			}
		} else {
			throw std::runtime_error("iop_address is empty");
		}
	} else {
		std::string msg = "iop_address[str] has invalid type ";
		msg += addr_param.get_type_name();
		throw std::runtime_error(msg.c_str());
	}
	RCLCPP_INFO(this->get_logger(), "Set JAUS address to: %d.%d.%d", p_id_subsystem, p_id_node, p_id_component);
	p_own_address = JausAddress(p_id_subsystem, p_id_node, p_id_component);
	p_publisher_diagnostics = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
	p_config_path = "nm.cfg";
	// read configuration from private parameter
	p_cfg->declare_param<std::string>("jaus_config", "nm.cfg", true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"Configuration path for JAUS library context", "");
	this->get_parameter("jaus_config", p_config_path);
	if (p_config_path.compare("nm.cfg") == 0) {
		char cwd[1024];
		if (getcwd(cwd, sizeof(cwd)) != nullptr) {
			RCLCPP_INFO(this->get_logger(), "JAUS configuration file: %s, pwd: %s", p_config_path.c_str(), cwd);
		} else {
			RCLCPP_INFO(this->get_logger(), "JAUS configuration file: %s", p_config_path.c_str());
		}
	} else {
		RCLCPP_INFO(this->get_logger(), "JAUS configuration file: %s", p_config_path.c_str());
	}
	p_cfg->declare_param<bool>("iop_use_remote_time", false, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
		"On false the timestamp of received messages will be overwritten by current time of local pc. Useful with if time is not synchronized and you got TF problem on local host.",
		"Default: false");
	this->get_parameter<bool>("iop_use_remote_time", p_use_remote_time);
	p_class_loader = nullptr;
	jausRouter = nullptr;
	// spawn another thread
	// p_connect_thread = new std::thread(std::bind(&iop::Component::p_connect_2_rte, this));
	// connect to RTE without new thread
	p_connect_2_rte();
}

Component::~Component()
{
	if (p_connect_thread != nullptr) {
		p_connect_thread->join();
		delete p_connect_thread;
	}

	p_discovery_client = std::shared_ptr<JTS::Service>();
	p_plugins_map.clear();
	service_list.clear();
	if (p_class_loader != nullptr)
		delete p_class_loader;
	if (jausRouter != nullptr)
		delete jausRouter;
	if (p_cfg != nullptr)
		delete p_cfg;
	std::cout << "Component deleted" << std::endl;
}

void Component::p_connect_2_rte()
{
	RCLCPP_INFO(this->get_logger(), "Connect to JAUS nodeManager...");
	send_diagnostic(3, "Connecting to RTE");
	this->jausRouter = new JTS::JausRouter(p_own_address, ieHandler, p_config_path);
	while (rclcpp::ok() && ! jausRouter->isConnected()) {
		sleep(1);
		delete this->jausRouter;
		send_diagnostic(2, "Timeout, connecting to RTE");
		this->jausRouter = new JTS::JausRouter(p_own_address, ieHandler, p_config_path);
	}
	if (rclcpp::ok() && jausRouter->isConnected()) {
		RCLCPP_INFO(this->get_logger(), "JAUS ID: %d", this->jausRouter->getJausAddress()->get());
		send_diagnostic(3, "Load Plug-Ins");
		load_plugins();
		start_component();
		send_diagnostic(0, "Started");
	}
}

void Component::send_diagnostic(int level, std::string message)
{
	diagnostic_msgs::msg::DiagnosticArray dmsg;
	dmsg.header.stamp = this->now();
	diagnostic_msgs::msg::DiagnosticStatus diag;
	diag.level = level;
	diag.name = this->get_fully_qualified_name();
	diag.message = message;
	dmsg.status.push_back(diag);
	p_publisher_diagnostics->publish(dmsg);
}

void Component::load_plugins()
{
	RCLCPP_INFO(this->get_logger(), "Load IOP plugin services specified by ~iop_services parameter:");
	p_discovery_client = std::shared_ptr<JTS::Service>();
	std::vector<std::string> plugin_names;
	bool has_range_sensor_service = false;
	bool has_visual_sensor_service = false;
	bool has_discovery_service = false;
	bool has_discovery_client_service = false;
	std::vector<std::string> plugins;
	p_cfg->declare_param<std::vector<std::string> >("iop_services", std::vector<std::string>(), true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
		"List of service to include into component.", "package_name/service_name");
	bool param_available = this->get_parameter("iop_services", plugins);
    	if (param_available) {
		for(unsigned int i = 0; i < plugins.size(); i++) {
			auto pkg_srv = iop::split(iop::trim(plugins[i]), '/');
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
				plugin_names.push_back(service);
			} else {
				RCLCPP_WARN(this->get_logger(), "skipped plugin entry '%s' because of invalid format", plugins[i]);
			}
		}
	} else {
        	std::string msg = "iop_services parameter not available! It should be a list of string with PACKAGE_NAME/SERVICE_NAME";
        	throw std::runtime_error(msg.c_str());
	}
	if (has_discovery_service & has_discovery_client_service) {
		RCLCPP_WARN(this->get_logger(), "In this version you do not need to include Discovery and DiscoverClient in the same component!");
		throw std::logic_error("include Discovery or DiscoverClient, not both!");
	}
	if (has_range_sensor_service & has_visual_sensor_service) {
		throw std::logic_error("You can not use RangeSensor{Client} and VisualSensor{Client} in the same component, since they use the same GeometricProperties message type!");
	}
	// determine paths for xml files with plugin description
	std::vector<std::string> p_xml_paths;
	p_class_loader = new pluginlib::ClassLoader<JTS::Service>("fkie_iop_component", "JTS::Service", std::string("plugin"), p_xml_paths);
	for (unsigned int i = 0; i < p_xml_paths.size(); ++i) {
		RCLCPP_WARN(this->get_logger(), p_xml_paths[i].c_str());
	}
	try {
		for (unsigned int i = 0; i < plugin_names.size(); ++i) {
			p_init_plugin(plugin_names[i], *p_class_loader);
		}
	} catch(pluginlib::PluginlibException& ex) {
		throw std::runtime_error("The plugin failed to load for some reason. Error: " + std::string(ex.what()));
	}
	// test for uninitialized pugins
	std::map<std::string, std::shared_ptr<JTS::Service> >::iterator it;
	for (it = p_plugins_map.begin(); it != p_plugins_map.end(); ++it) {
		if (!it->second.get()->isInitialized()) {
			throw std::runtime_error("Not initialized: " + it->second.get()->getName());
		}
	}
	RCLCPP_INFO(this->get_logger(), "... plugin loading complete!");
	// register the services
	if (p_discovery_client.get() != nullptr) {
		RCLCPP_INFO(this->get_logger(), "Add %d services to register by discovery service...", p_plugins_map.size());
		std::map<std::string, std::shared_ptr<JTS::Service> >::iterator it2;
		for (it2 = p_plugins_map.begin(); it2 != p_plugins_map.end(); ++it2) {
			p_discovery_client->registerService(it2->second->getURN(), it2->second->getVersionManjor(), it2->second->getVersionMinor(), *this->jausRouter->getJausAddress());
		}
		RCLCPP_INFO(this->get_logger(), "... all services added");
	} else {
		RCLCPP_WARN(this->get_logger(), "Discovery nor DiscoveryClient not found, the services are neither registered nor discovered!");
	}
	RCLCPP_INFO(this->get_logger(), "... initialization complete");
}

std::shared_ptr<JTS::Service> Component::p_init_plugin(std::string name, pluginlib::ClassLoader<JTS::Service>& class_loader)
{
	if (name.empty()) {
		return std::shared_ptr<JTS::Service>();
	}
	std::map<std::string, std::shared_ptr<JTS::Service> >::iterator it = p_plugins_map.find(name);
	if (it != p_plugins_map.end()) {
		return it->second;
	} else {
		std::shared_ptr<JTS::Service> plugin = class_loader.createSharedInstance(name);
		std::shared_ptr<JTS::Service> base_plugin;
		if (!plugin->getNameInheritsFrom().empty()) {
			it = p_plugins_map.find(plugin->getNameInheritsFrom());
			if (it == p_plugins_map.end()) {
				base_plugin = p_init_plugin(plugin->getNameInheritsFrom(), class_loader);
			} else {
				base_plugin = it->second;
			}
			if (base_plugin == nullptr) {
				throw std::logic_error("required plugin " + plugin->getNameInheritsFrom() + " for service '" + name + "' not found");
			}
		} else {
			//plugin->init_service(this, this->jausRouter);
		}
		RCLCPP_INFO(this->get_logger().get_child("PluginLoader"), "=== Initialize IOP-plugin %s === %s ===", name.c_str(), plugin->getURN().c_str());
		plugin->init_service(std::dynamic_pointer_cast<iop::Component>(shared_from_this()), this->jausRouter, base_plugin.get());
		if (plugin->getName().compare("DiscoveryService") == 0 || plugin->getName().compare("DiscoveryClientService") == 0) {
			p_discovery_client = plugin;
		}
		p_plugins_map[name] = plugin;
		service_list.push_back(plugin);
		return plugin;
	}
	return std::shared_ptr<JTS::Service>();
}

bool Component::has_service(std::string service_uri)
{
	std::map<std::string, std::shared_ptr<JTS::Service> >::iterator it;
	for (it = p_plugins_map.begin(); it != p_plugins_map.end(); ++it) {
		if (it->second.get()->getURN().compare(service_uri) == 0) {
			return true;
		}
	}
	return false;
}

void Component::start_component()
{
	std::shared_ptr<JTS::Service> service;

	jausRouter->start();
	this->start();

	RCLCPP_INFO(this->get_logger(), "Start Services...");
	for (unsigned int i = 0; i < service_list.size(); i++)
	{
		service = service_list.at(i);
		RCLCPP_INFO(this->get_logger(), " > %s", service->getName().c_str());
		service->start();
	}
	RCLCPP_INFO(this->get_logger(), "Start Services...done!");

}


void Component::shutdown_component()
{
	std::shared_ptr<JTS::Service> service;

	std::cout << "Shutdown component..." << std::endl;
	for (unsigned int i = 0; i < service_list.size(); i++)
	{
		service = service_list.at(i);
		std::cout << "  stop " << service->getName() << " ..."<< std::endl;
		service->stop();
	}

	this->stop();
	jausRouter->stop();
	delete this->jausRouter;

	// remove all plugins
	p_discovery_client = std::shared_ptr<JTS::Service>();
	p_plugins_map.clear();
	service_list.clear();
	std::cout << "Shutdown component finished" << std::endl;
}


JTS::Service* Component::get_service(std::string service_name)
{
	// extend if given name has no "Service" at the end
	std::string ending = "Service";
	if (service_name.size() < 8 || !std::equal(ending.rbegin(), ending.rend(), service_name.rbegin())) {
		service_name += "Service";
	}
	std::map<std::string, std::shared_ptr<JTS::Service> >::iterator it;
	for (it = p_plugins_map.begin(); it != p_plugins_map.end(); ++it) {
		if (it->second.get()->getName().compare(service_name) == 0
				&& it->second.get()->isInitialized()) {
			return it->second.get();
		}
	}
	return NULL;
}


std::shared_ptr<ocu::Slave> Component::get_slave()
{
	return p_slave;
}

void Component::set_slave(std::shared_ptr<ocu::Slave> slave)
{
	p_slave = slave;
}

int64_t Component::now_secs()
{
        auto now = std::chrono::steady_clock::now();
        return std::chrono::time_point_cast<std::chrono::seconds>(now).time_since_epoch().count();
}

int64_t Component::now_millis()
{
        auto now = std::chrono::steady_clock::now();
        return std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
}

Timestamp Component::from_iop(uint64_t days, uint64_t hours, uint64_t minutes, uint64_t seconds, uint64_t milliseconds)
{
	return Timestamp(days, hours, minutes, seconds, milliseconds, this->now(), p_use_remote_time);
}

Timestamp Component::from_ros(rclcpp::Time ros_time)
{
	return Timestamp(ros_time);
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
				RCLCPP_WARN(this->get_logger(), "Error while cast JAUS message to right transport version, result is 0");
			} else {
				done = service_list.at(i-1)->processTransitions(ie);
				if (done) {
					Receive* casted_ie = (Receive*) ie;
					unsigned short id = *((unsigned short*) casted_ie->getBody()->getReceiveRec()->getMessagePayload()->getData());
					RCLCPP_DEBUG(this->get_logger().get_child("InternalProcess"), "PROCESSED: %s - message type: %x, transition: %s", service_list.at(i-1)->getURN().c_str(), id, casted_ie->getName().c_str());
				}
			}
		}
	}
	for (unsigned int i = service_list.size(); i>0; i--)
	{
		if (!done) {
			if (ie == 0) {
				RCLCPP_WARN(this->get_logger(), "Error while cast JAUS message to right transport version, result is 0");
			} else {
				done = service_list.at(i-1)->defaultTransitions(ie);
				if (done) {
					Receive* casted_ie = (Receive*) ie;
					unsigned short id = *((unsigned short*) casted_ie->getBody()->getReceiveRec()->getMessagePayload()->getData());
					RCLCPP_DEBUG(this->get_logger().get_child("InternalProcess"), "PROCESSED DEFAULT: %s - message type: %x, transition: %s", service_list.at(i-1)->getURN().c_str(), id, casted_ie->getName().c_str());
				}
			}
		}
	}
	if (!done) {
		Receive* casted_ie = (Receive*) ie;
		unsigned short id = *((unsigned short*) casted_ie->getBody()->getReceiveRec()->getMessagePayload()->getData());
		RCLCPP_DEBUG(this->get_logger().get_child("InternalProcess"), "NOT PROCESSED: %x, transition: %s, source: %s", id, ie->getName().c_str(), ie->getSource().c_str());
	}
}
