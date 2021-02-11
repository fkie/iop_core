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

#pragma once

#include "rclcpp/node.hpp"


namespace iop {

class Component;
class RosNode: public rclcpp::Node
{
  public:
    RosNode(const std::string &node_name, const std::string &namespace_);
    ~RosNode();

    static RosNode& get_instance()
    {
        if (global_ptr == nullptr) throw std::runtime_error("RosNode was not initialized!");
        return *global_ptr;
    }
    Component* init_component(int subsystem, int node, int component);
    Component* get_component();

  private:
    static RosNode* global_ptr;
    int p_id_subsystem ;
    int p_id_node;
    int p_id_component;
    bool p_search_for_id_params;
    bool p_iop_initialized;
    Component* p_component;
};

}
