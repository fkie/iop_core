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


#ifndef ROSINIT_H
#define ROSINIT_H

#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>

static ros::NodeHandle *nh;
static ros::NodeHandle *pnh;

namespace iop {

template <class T>
T *ros_init(int argc, char *argv[], const std::string& ros_node_name, int subsystem, int node, int component){
    ros::init(argc, argv, ros_node_name);
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
       ros::console::notifyLoggerLevelsChanged();
    }
    // Get the parameter for component identification
    int id_subsystem = subsystem;
    int id_node = node;
    int id_component = component;
    bool search_for_id_params = true;
    nh = new ros::NodeHandle();
    pnh = new ros::NodeHandle("~");
    std::string iop_address;
    float iop_address_float;
    int iop_address_int;
    if (pnh->getParam("iop_address", iop_address)) {
      int p1, p2, p3;
      int scan_result = std::sscanf(iop_address.c_str(), "%d.%d.%d", &p1, &p2, &p3);
      if (scan_result == 2) {
        ROS_INFO("found ~iop_address: %s", iop_address.c_str());
        id_subsystem = p1;
        id_node = p2;
        search_for_id_params = false;
      } else if (scan_result == 3) {
        ROS_INFO("found ~iop_address: %s", iop_address.c_str());
        id_subsystem = p1;
        id_node = p2;
        id_component = p3;
        search_for_id_params = false;
      } else {
        ROS_WARN("invalid format in ~iop_address[str]: %s, should be subsystem.node.component or subsystem.node", iop_address.c_str());
      }
    } else if (pnh->getParam("iop_address", iop_address_float)) {
      throw std::runtime_error("found ~iop_address, but with invalid type: float");
    } else if (pnh->getParam("iop_address", iop_address_int)) {
      throw std::runtime_error("found ~iop_address, but with invalid type: int");
    } else {
      ROS_INFO("~iop_address[str] not found, search for id_subsystem, id_node and id_component...");
    }
    // if no ~iop_address was found or it was invalid, we search for id_.. parameter
    if (search_for_id_params) {
      if (pnh->getParam("id_subsystem", id_subsystem)) {
        ROS_INFO("found ~id_subsystem: %d", id_subsystem);
      } else if (nh->getParam("id_subsystem", id_subsystem)) {
        ROS_INFO("found id_subsystem: %d", id_subsystem);
      }
      if (pnh->getParam("id_node", id_node)) {
        ROS_INFO("found ~id_node: %d", id_node);
      } else if (nh->getParam("id_node", id_node)) {
        ROS_INFO("found id_node: %d", id_node);
      }
      if (pnh->getParam("id_component", id_component)) {
        ROS_INFO("found ~id_component: %d", id_component);
      } else if (nh->getParam("id_component", id_component)) {
        ROS_INFO("found id_component: %d", id_component);
      }
    }
    ROS_INFO("JAUS address of the component: %d.%d.%d", id_subsystem, id_node, id_component);
    return new T(id_subsystem, id_node, id_component);
}

};
#endif // ROSINIT_H
