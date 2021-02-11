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

#include <stdio.h>
#include "fkie_iop_component/ros_node.hpp"
#include "fkie_iop_component/iop_component.h"


using namespace iop;
RosNode* RosNode::global_ptr = nullptr;

RosNode::RosNode(const std::string &node_name, const std::string &namespace_): 
    Node(node_name, namespace_,
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true))
{
    global_ptr = this;
    p_id_subsystem = 0;
    p_id_node = 0;
    p_id_component = 0;
    p_search_for_id_params = true;
    p_iop_initialized = false;
    p_component = nullptr;
	rcl_interfaces::msg::ParameterDescriptor addr_description;
	addr_description.name = "iop_address";
	addr_description.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
	addr_description.description = "Address of the IOP component";
	addr_description.read_only = true;
	addr_description.additional_constraints = "{subsystem-65535}.{node-255}.{component-255}";
    this->declare_parameter<std::string>("iop_address", "", addr_description);
}

RosNode::~RosNode()
{
    if (p_component != nullptr) {
        delete p_component;
        p_component = nullptr;
    }
    global_ptr = nullptr;
}

Component* RosNode::init_component(int subsystem, int node, int component) {
    if (p_component != nullptr) {
        RCLCPP_WARN(this->get_logger(), "IOP component already initialized");
        return p_component;
    }
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
    // if no ~iop_address was found or it was invalid, we search for id_.. parameter
    // if (search_for_id_params) {
    //   if (pnh->getParam("id_subsystem", id_subsystem)) {
    //     ROS_INFO("found ~id_subsystem: %d", id_subsystem);
    //   } else if (nh->getParam("id_subsystem", id_subsystem)) {
    //     ROS_INFO("found id_subsystem: %d", id_subsystem);
    //   }
    //   if (pnh->getParam("id_node", id_node)) {
    //     ROS_INFO("found ~id_node: %d", id_node);
    //   } else if (nh->getParam("id_node", id_node)) {
    //     ROS_INFO("found id_node: %d", id_node);
    //   }
    //   if (pnh->getParam("id_component", id_component)) {
    //     ROS_INFO("found ~id_component: %d", id_component);
    //   } else if (nh->getParam("id_component", id_component)) {
    //     ROS_INFO("found id_component: %d", id_component);
    //   }
    // }
    RCLCPP_INFO(this->get_logger(), "JAUS address of the component: %d.%d.%d", p_id_subsystem, p_id_node, p_id_component);
    p_component = new Component(p_id_subsystem, p_id_node, p_id_component, *this);
    return p_component;
}

Component* RosNode::get_component()
{
    return p_component;
}
