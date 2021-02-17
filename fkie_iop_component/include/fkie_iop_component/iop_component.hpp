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

#include <thread>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <class_loader/multi_library_class_loader.hpp>
#include "Service.h"
#include "EventReceiver.h"
#include "Transport/JausTransport.h"
#include "InternalEvents/InternalEvent.h"

#ifndef PLUGINLIB__DISABLE_BOOST_FUNCTIONS
#define PLUGINLIB__DISABLE_BOOST_FUNCTIONS
#endif
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace iop
{

    class Config;
    class Component: public rclcpp::Node, public JTS::EventReceiver
    {
    public:
        Component(const std::string node_name, const std::string namespace_);
        void init(unsigned int subsystem, unsigned short node, unsigned short component);
        virtual ~Component();

        rclcpp::Logger roslog(const std::string& suffix) { return get_logger().get_child(suffix); }
        bool has_service(std::string service_uri);
        void start_component();
        void shutdown_component();
        // level: 0-OK, 1-WARNING, 2-ERROR, 3 STALE
        void send_diagnostic(int level, std::string message);

        JTS::Service* get_service(std::string service_name);
        void p_connect_2_rte();
    protected:
        virtual void processInternalEvent(JTS::InternalEvent* ie);

        iop::Config* p_cfg;
        JausAddress p_own_address;
        std::shared_ptr<JTS::Service> p_discovery_client;
        std::thread* p_connect_thread;
        pluginlib::ClassLoader<JTS::Service>* p_class_loader;
        std::map<std::string, std::shared_ptr<JTS::Service> > p_plugins_map;
        std::vector<std::shared_ptr<JTS::Service>> service_list;
        JTS::JausRouter* jausRouter;
        std::string p_config_path;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr p_publisher_diagnostics;
        int p_id_subsystem ;
        int p_id_node;
        int p_id_component;
        bool p_search_for_id_params;
        bool p_iop_initialized;


        void load_plugins();
        std::shared_ptr<JTS::Service> p_init_plugin(std::string name, pluginlib::ClassLoader<JTS::Service>& class_loader);
    };
}
#endif // IOP_COMPONENT_H
