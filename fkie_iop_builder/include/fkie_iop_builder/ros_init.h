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


// #ifndef ROSINIT_H
// #define ROSINIT_H

// #include <stdio.h>
// #include "rclcpp/node.hpp"


// namespace iop {


// class ComponentNode: public rclcpp::Node
// {
//   public:
//       ComponentNode(const std::string &node_name, const std::string &namespace_, int subsystem, int node, int component): 
//           Node(node_name, namespace_,
//               rclcpp::NodeOptions()
//                   .allow_undeclared_parameters(true)
//                   .automatically_declare_parameters_from_overrides(true))
//       {
//           id_subsystem = subsystem;
//           id_node = node;
//           id_component = component;
//           this->declare_parameter<std::string>("iop_address", "");
//       }

//       template <class T>
//       T *init_component(int subsystem, int node, int component) {
//           rclcpp::Parameter addr_param = this->get_parameter("iop_address");
//           if (addr_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
//               std::string addr_str = addr_param.as_string();
//               if (!addr_str.empty()) {
//                   int p1, p2, p3;
//                   int scan_result = std::sscanf(addr_str.c_str(), "%d.%d.%d", &p1, &p2, &p3);
//                   if (scan_result == 2) {
//                       RCLCPP_INFO("found iop_address: %s", addr_str.c_str());
//                       id_subsystem = p1;
//                       id_node = p2;
//                       search_for_id_params = false;
//                   } else if (scan_result == 3) {
//                       RCLCPP_INFO("found iop_address: %s", addr_str.c_str());
//                       id_subsystem = p1;
//                       id_node = p2;
//                       id_component = p3;
//                       search_for_id_params = false;
//                   } else {
//                       RCLCPP_WARN("invalid format in iop_address[str]: %s, should be subsystem.node.component or subsystem.node", addr_str.c_str());
//                   }
//               } else {
//                   throw std::runtime_error("iop_address is empty");
//               }
//           } else {
//               throw std::runtime_error("iop_address[str] has invalid type %s", addr_param.get_type_name().c_str());
//           }
//           // if no ~iop_address was found or it was invalid, we search for id_.. parameter
//           // if (search_for_id_params) {
//           //   if (pnh->getParam("id_subsystem", id_subsystem)) {
//           //     ROS_INFO("found ~id_subsystem: %d", id_subsystem);
//           //   } else if (nh->getParam("id_subsystem", id_subsystem)) {
//           //     ROS_INFO("found id_subsystem: %d", id_subsystem);
//           //   }
//           //   if (pnh->getParam("id_node", id_node)) {
//           //     ROS_INFO("found ~id_node: %d", id_node);
//           //   } else if (nh->getParam("id_node", id_node)) {
//           //     ROS_INFO("found id_node: %d", id_node);
//           //   }
//           //   if (pnh->getParam("id_component", id_component)) {
//           //     ROS_INFO("found ~id_component: %d", id_component);
//           //   } else if (nh->getParam("id_component", id_component)) {
//           //     ROS_INFO("found id_component: %d", id_component);
//           //   }
//           // }
//           RCLCPP_INFO("JAUS address of the component: %d.%d.%d", id_subsystem, id_node, id_component);
//           return new T(id_subsystem, id_node, id_component);
//       }


//     private:
//         int id_subsystem = 0;
//         int id_node = 0;
//         int id_component = 0;
//         bool search_for_id_params = true;
// }

// #endif // ROSINIT_H
