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

#include <rclcpp/rclcpp.hpp>
#include "fkie_iop_component/iop_component.hpp"
#include "fkie_iop_component/iop_config.hpp"


using namespace iop;

Config::Config(std::shared_ptr<iop::Component> cmp, std::string ns)
: p_cmp(cmp), p_ns(ns)
{
}

std::string Config::get_topic_name(const std::string& name, const std::string& prefix)
{
    std::string result;
    std::string param_name = prefix;
    if (name[0] == '/') {
        param_name += name.substr(1);
    } else {
        param_name += name;
    }
    std::replace(param_name.begin(), param_name.end(), '/', '_');
    declare_param<std::string>(param_name, name, true,
        rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
        "", "");
    // only from private namespace
    param<std::string>(param_name, result, name, true);
    return result;
}
