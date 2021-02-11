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

#include <algorithm>
#include <map>
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include "fkie_iop_component/iop_config.h"


using namespace iop;

Config::Config(std::string ns)
{
    p_ns = ns;
    p_rosnode = &iop::RosNode::get_instance();
}

template<typename T>
void Config::param(std::string param_name, T &param_val, T default_val,
        bool from_private,
        std::string unit,
        std::map<int, std::string> type_map)
{
    std::string got_from;
    std::string with_ns = p_ns + "/" + param_name;
    if (p_rosnode->has_parameter(with_ns)) {
        p_rosnode->get_parameter_or(param_name, param_val, default_val);
        got_from = p_ns;
    } else if (from_private && p_rosnode->has_parameter(param_name)) {
        p_rosnode->get_parameter_or(param_name, param_val, default_val);
        got_from = "~";
    } else {
        p_rosnode->get_parameter_or(with_ns, param_val, default_val);
        got_from = "default";
    }
    //RCLCPP_INFO(p_rosnode->get_logger(), "ROS param[" << p_ns << "]: " << param_name << " = " << tostr(param_val, unit, type_map) << " [ns: " << got_from << "]");
    RCLCPP_INFO(p_rosnode->get_logger(), "ROS param[%s]: %s = %s [ns: %s]", p_ns.c_str(), param_name.c_str(), tostr(param_val, unit, type_map).c_str(), got_from.c_str());
}


template<typename MessageT, typename AllocatorT, typename PublisherT>
std::shared_ptr<PublisherT> Config::create_publisher(
        const std::string& topic_name,
        const rclcpp::QoS& qos,
        const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options)
{
    std::string name = get_topic_name(topic_name, "topic_pub_");
    auto result = p_rosnode->create_publisher(name, qos, options);
    RCLCPP_INFO(p_rosnode->get_logger(), "ROS publisher[%s]: %s <%s>", p_ns.c_str(), result.get_topic_name(), typeid(MessageT).name());
    return result;
}

template<typename MessageT, typename CallbackT, typename AllocatorT, typename CallbackMessageT, typename SubscriptionT, typename MessageMemoryStrategyT>
std::shared_ptr<SubscriptionT> Config::create_subscription(
        const std::string & topic_name,
        const rclcpp::QoS & qos,
        CallbackT && callback,
        const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options,
        typename MessageMemoryStrategyT::SharedPtr msg_mem_strat)
{
    std::string name = get_topic_name(topic_name, "topic_sub_");
    auto result = p_rosnode->create_subscription(name, qos, std::forward<CallbackT>(callback), options, msg_mem_strat);
    RCLCPP_INFO(p_rosnode->get_logger(), "ROS subscriber[%s]: %s <%s>", p_ns.c_str(), result.get_topic_name(), typeid(MessageT).name());
    return result;
}

template<typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr Config::create_service(
        const std::string & service_name,
        CallbackT && callback,
        const rmw_qos_profile_t & qos_profile,
        rclcpp::CallbackGroup::SharedPtr group)
{
    std::string name = get_topic_name(service_name, "topic_svr_");
    auto result = p_rosnode->create_service(name, std::forward<CallbackT>(callback), qos_profile, group);
    RCLCPP_INFO(p_rosnode->get_logger(), "ROS service[%s]: %s <%s>", p_ns.c_str(), result.get_service_name(), typeid(ServiceT).name());
    return result;
}

std::string Config::tostr(char& param_val, std::string unit, std::map<int, std::string>& type_map)
{
    std::ostringstream result;
    typename std::map<int, std::string>::const_iterator it = type_map.find((int)param_val);
    if (it != type_map.end()) {
        if (!unit.empty()) {
            result << (int)param_val << " " << unit << " (" << it->second << ") ";
        } else {
            result << (int)param_val << " (" << it->second << ") ";
        }
    } else {
        result << (int)param_val;
        if (!unit.empty()) {
            result << " " << unit;
        }
    }
    return result.str();
}

std::string Config::tostr(unsigned char& param_val, std::string unit, std::map<int, std::string>& type_map)
{
    std::ostringstream result;
    typename std::map<int, std::string>::const_iterator it = type_map.find((int)param_val);
    if (it != type_map.end()) {
        if (!unit.empty()) {
            result << (int)param_val << " " << unit << " (" << it->second << ") ";
        } else {
            result << (int)param_val << " (" << it->second << ") ";
        }
    } else {
        result << (int)param_val;
        if (!unit.empty()) {
            result << " " << unit;
        }
    }
    return result.str();
}

std::string Config::tostr(int& param_val, std::string unit, std::map<int, std::string>& type_map)
{
    std::ostringstream result;
    typename std::map<int, std::string>::const_iterator it = type_map.find(param_val);
    if (it != type_map.end()) {
        if (!unit.empty()) {
            result << param_val << " " << unit << " (" << it->second << ") ";
        } else {
            result << param_val << " (" << it->second << ") ";
        }
    } else {
        result << param_val;
        if (!unit.empty()) {
            result << " " << unit;
        }
    }
    return result.str();
}

std::string Config::tostr(double& param_val, std::string unit, std::map<int, std::string>& /* type_map */)
{
    std::ostringstream result;
    result << param_val;
    if (!unit.empty()) {
        result << " " << unit;
    }
    return result.str();
}

std::string Config::tostr(float& param_val, std::string unit, std::map<int, std::string>& /* type_map */)
{
    std::ostringstream result;
    result << param_val;
    if (!unit.empty()) {
        result << " " << unit;
    }
    return result.str();
}

std::string Config::tostr(bool& param_val, std::string unit, std::map<int, std::string>& /* type_map */)
{
    std::ostringstream result;
    result << param_val;
    if (!unit.empty()) {
        result << " " << unit;
    }
    return result.str();
}

std::string Config::tostr(std::string& param_val, std::string unit, std::map<int, std::string>& /* type_map */)
{
    if (!unit.empty()) {
        return param_val + " " + unit;
    }
    return param_val;
}

std::string Config::tostr(std::vector<int>& param_val, std::string unit, std::map<int, std::string>& type_map)
{
    std::ostringstream result;
    result << "[";
    int i = 0;
    for (std::vector<int>::iterator it = param_val.begin(); it != param_val.end(); ++it) {
        if (i > 0) {
            result << ", ";
        }
        i++;
        result << *it;
        typename std::map<int, std::string>::const_iterator itt = type_map.find(*it);
        if (itt != type_map.end()) {
            result << " (" << itt->second << ") ";
        }
    }
    result << "]";
    if (!unit.empty()) {
        result << " " << unit;
    }
    return result.str();
}

std::string Config::tostr(std::vector<double>& param_val, std::string unit, std::map<int, std::string>& /* type_map */)
{
    std::ostringstream result;
    result << "[";
    int i = 0;
    for (std::vector<double>::iterator it = param_val.begin(); it != param_val.end(); ++it) {
        if (i > 0) {
            result << ", ";
        }
        i++;
        result << *it;
    }
    result << "]";
    if (!unit.empty()) {
        result << " " << unit;
    }
    return result.str();
}

std::string Config::tostr(std::vector<float>& param_val, std::string unit, std::map<int, std::string>& /* type_map */)
{
    std::ostringstream result;
    result << "[";
    int i = 0;
    for (std::vector<float>::iterator it = param_val.begin(); it != param_val.end(); ++it) {
        if (i > 0) {
            result << ", ";
        }
        i++;
        result << *it;
    }
    result << "]";
    if (!unit.empty()) {
        result << " " << unit;
    }
    return result.str();
}

std::string Config::tostr(std::vector<std::string>& param_val, std::string unit, std::map<int, std::string>& /* type_map */)
{
    std::ostringstream result;
    result << "[";
    int i = 0;
    for (std::vector<std::string>::iterator it = param_val.begin(); it != param_val.end(); ++it) {
        if (i > 0) {
            result << ", ";
        }
        i++;
        result << *it;
    }
    result << "]";
    if (!unit.empty()) {
        result << " " << unit;
    }
    return result.str();
}

std::string Config::get_topic_name(std::string name, std::string prefix)
{
    std::string result;
    std::string param_name = prefix;
    if (name[0] == '/') {
        param_name += name.substr(1);
    } else {
        param_name += name;
    }
    std::replace(param_name.begin(), param_name.end(), '/', '_');
    // only from private namespace
    param(param_name, result, name, true);
    return result;
}
