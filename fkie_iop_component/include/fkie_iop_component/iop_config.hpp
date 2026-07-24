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

#ifndef IOP_CONFIG_H
#define IOP_CONFIG_H

#include "iop_component.hpp"
#include <algorithm>
#include <map>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <string>
#include <type_traits>

namespace iop {

// Helper trait to detect iterable containers
template <typename T, typename = void>
struct is_iterable : std::false_type { };

template <typename T>
struct is_iterable<T, std::void_t<decltype(std::declval<T>().begin()), decltype(std::declval<T>().end())>> : std::true_type { };

class Config {
public:
    Config(std::shared_ptr<iop::Component> cmp, std::string ns = "~");

    /** Wrapper for get parameter with default options **/
    template <typename T>
    void param(std::string param_name, T& param_val, const T& default_val,
        bool read_only = true,
        uint8_t ptype = rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
        std::string description = "",
        std::string additional_constraints = "",
        std::string unit = "")
    {
        std::string got_from = get_param<T>(param_name, param_val, default_val, read_only, ptype, description, additional_constraints);
        RCLCPP_INFO(p_cmp->get_logger(), "[%s] ROS param: %s = %s [ns: %s]", p_ns.c_str(), param_name.c_str(), str_val_unit(param_val, unit).c_str(), got_from.c_str());
    }

    /** Wrapper for get parameter with default options **/
    template <typename T>
    void param_vector(std::string param_name, T& param_val, const T& default_val,
        bool read_only = true,
        uint8_t ptype = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY,
        std::string description = "",
        std::string additional_constraints = "",
        std::string unit = "")
    {
        std::string got_from = get_param<T>(param_name, param_val, default_val, read_only, ptype, description, additional_constraints);
        std::string vstr = str_vector(param_val);
        RCLCPP_INFO(p_cmp->get_logger(), "[%s] ROS param: %s = %s [ns: %s]", p_ns.c_str(), param_name.c_str(), str_val_unit(vstr, unit).c_str(), got_from.c_str());
    }

    /** Wrapper for get parameter with default options **/
    template <typename T>
    void param_named(std::string param_name, T& param_val, const T& default_val,
        std::map<T, std::string> type_map,
        bool read_only = true,
        uint8_t ptype = rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
        std::string description = "",
        std::string additional_constraints = "",
        std::string unit = "")
    {
        std::string got_from = get_param<T>(param_name, param_val, default_val, read_only, ptype, description, additional_constraints);
        std::string vname = str_from_map(param_val, type_map);
        RCLCPP_INFO(p_cmp->get_logger(), "[%s] ROS param: %s = %s%s[ns: %s]", p_ns.c_str(), param_name.c_str(), str_val_unit(param_val, unit).c_str(), vname.c_str(), got_from.c_str());
    }

    /** Wrapper for create_publisher **/
    template <
        typename MessageT,
        typename AllocatorT = std::allocator<void>,
        typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>>
    std::shared_ptr<PublisherT> create_publisher(
        const std::string& topic_name,
        const rclcpp::QoS& qos,
        const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options = rclcpp::PublisherOptionsWithAllocator<AllocatorT>())
    {
        std::string name = get_topic_name(topic_name, "topic.pub.");
        auto result = p_cmp->create_publisher<MessageT>(name, qos, options);
        RCLCPP_INFO(p_cmp->get_logger(), "[%s] ROS publisher: %s [type: %s]", p_ns.c_str(), result->get_topic_name(), typeid(MessageT).name());
        return result;
    }

    /** Wrapper for create_subscription **/
    template <
        typename MessageT,
        typename CallbackT,
        typename AllocatorT = std::allocator<void>,
        typename CallbackMessageT =
            typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
        typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>,
        typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
            CallbackMessageT,
            AllocatorT>>
    std::shared_ptr<SubscriptionT> create_subscription(
        const std::string& topic_name,
        const rclcpp::QoS& qos,
        CallbackT&& callback,
        const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options = rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(),
        typename MessageMemoryStrategyT::SharedPtr msg_mem_strat = (MessageMemoryStrategyT::create_default()))
    {
        std::string name = get_topic_name(topic_name, "topic.sub.");
        auto qos_cfg = qos;
        auto rmw_qos = qos.get_rmw_qos_profile();
        bool default_rl = rmw_qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        bool reliable = get_param_by_topic_name(topic_name, "topic.sub.qos.reliable.", default_rl, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL);
        if (reliable) {
            qos_cfg = qos_cfg.reliable();
        } else {
            qos_cfg = qos_cfg.best_effort();
        }
        bool default_tl = rmw_qos.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        bool transient_local = get_param_by_topic_name(topic_name, "topic.sub.qos.transient_local.", default_tl, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL);
        if (transient_local) {
            qos_cfg = qos_cfg.transient_local();
        } else {
            qos_cfg = qos_cfg.durability_volatile();
        }

        auto result = p_cmp->create_subscription<MessageT>(name, qos_cfg, std::forward<CallbackT>(callback), options, msg_mem_strat);
        RCLCPP_INFO(p_cmp->get_logger(), "[%s] ROS subscriber: %s [type: %s, reliable: %d, transient_local: %d]", p_ns.c_str(), result->get_topic_name(), typeid(MessageT).name(), reliable, transient_local);
        return result;
    }

    /** Wrapper for create_service **/
    template <typename ServiceT, typename CallbackT>
    typename rclcpp::Service<ServiceT>::SharedPtr
    create_service(
        const std::string& service_name,
        CallbackT&& callback,
        const rclcpp::QoS& qos_profile = rclcpp::ServicesQoS(),
        rclcpp::CallbackGroup::SharedPtr group = nullptr)
    {
        std::string name = get_topic_name(service_name, "topic.svr.");
        auto result = p_cmp->create_service<ServiceT>(name, std::forward<CallbackT>(callback), qos_profile, group);
        RCLCPP_INFO(p_cmp->get_logger(), "[%s] ROS service: %s [type: %s]", p_ns.c_str(), result->get_service_name(), typeid(ServiceT).name());
        return result;
    }

    template <typename T>
    std::string str_from_map(T& param_val, std::map<T, std::string>& type_map)
    {
        std::ostringstream result;
        typename std::map<T, std::string>::const_iterator it = type_map.find(param_val);
        if (it != type_map.end()) {
            result << " (" << it->second << ") ";
        }
        return result.str();
    }

    template <typename T>
    std::string str_vector(T& param_val)
    {
        std::ostringstream result;
        std::copy(param_val.cbegin(), param_val.cend(), std::ostream_iterator<typename T::value_type>(result, ", "));
        return result.str();
    }

    template <typename T>
    std::string str_val_unit(T& param_val, std::string& unit)
    {
        std::ostringstream result;
        if constexpr (std::is_same_v<T, int8_t> || std::is_same_v<T, uint8_t>) {
            result << static_cast<int>(param_val);
        } else if constexpr (std::is_same_v<T, double>) {
            result.precision(6);
            result << param_val;
        } else if constexpr (std::is_same_v<T, std::string>) {
            result << param_val;
        } else if constexpr (is_iterable<T>::value) {
            // Container types (e.g. std::vector)
            result << "[";
            bool first = true;
            for (const auto& v : param_val) {
                if (!first)
                    result << ", ";
                result << v;
                first = false;
            }
            result << "]";
        } else {
            result << param_val;
        }
        if (!unit.empty()) {
            result << " " << unit;
        }
        return result.str();
    }

protected:
    std::shared_ptr<iop::Component> p_cmp;
    std::string p_ns;
    std::string get_topic_name(const std::string& name, const std::string& prefix);
    template <typename T>
    T get_param_by_topic_name(const std::string& name, const std::string& prefix,
        const T& default_value,
        uint8_t ptype = rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
        std::string description = "")
    {
        T result;
        std::string param_name = prefix;
        if (name[0] == '/') {
            param_name += name.substr(1);
        } else {
            param_name += name;
        }
        std::replace(param_name.begin(), param_name.end(), '/', '_');
        param<T>(param_name, result, default_value);
        return result;
    }

    template <typename T>
    std::string get_param(std::string param_name, T& param_val, const T& default_val,
        bool read_only = true,
        uint8_t ptype = rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
        std::string description = "",
        std::string additional_constraints = "")
    {
        std::string got_from = "default";
        std::string with_ns = p_ns.empty() ? param_name : p_ns + "." + param_name;

        // Step 1: Temporarily declare without default to check if externally set
        bool externally_set_ns = false;
        bool externally_set_priv = false;

        try {
            if (!p_cmp->has_parameter(with_ns)) {
                rcl_interfaces::msg::ParameterDescriptor desc;
                desc.dynamic_typing = true;
                p_cmp->declare_parameter(with_ns, rclcpp::ParameterValue { }, desc);
            }
            rclcpp::Parameter p = p_cmp->get_parameter(with_ns);
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
                param_val = p.get_value<T>();
                externally_set_ns = true;
                got_from = p_ns;
            }
        } catch (...) {
        }

        if (!externally_set_ns && param_name != with_ns) {
            try {
                if (!p_cmp->has_parameter(param_name)) {
                    rcl_interfaces::msg::ParameterDescriptor desc;
                    desc.dynamic_typing = true;
                    p_cmp->declare_parameter(param_name, rclcpp::ParameterValue { }, desc);
                }
                rclcpp::Parameter p = p_cmp->get_parameter(param_name);
                if (p.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
                    param_val = p.get_value<T>();
                    externally_set_priv = true;
                    got_from = "~";
                }
            } catch (...) {
            }
        }

        // If nothing was externally set, use the default value
        if (!externally_set_ns && !externally_set_priv) {
            param_val = default_val;
            got_from = "default";
        }

        // Step 2: Undeclare and re-declare with proper default and descriptor
        try {
            if (p_cmp->has_parameter(with_ns)) {
                p_cmp->undeclare_parameter(with_ns);
            }
        } catch (...) {
        }

        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.name = with_ns;
        descriptor.type = ptype;
        descriptor.description = description;
        descriptor.read_only = read_only;
        descriptor.additional_constraints = additional_constraints;
        try {
            p_cmp->declare_parameter<T>(with_ns, default_val, descriptor);
            // Set the current value (externally provided or default)
            if (externally_set_ns || externally_set_priv) {
                p_cmp->set_parameter(rclcpp::Parameter(with_ns, param_val));
            }
        } catch (const std::exception& ex) {
            std::cout << "ERR get_param declare:" << ex.what() << std::endl;
        }

        return got_from;
    }
};
}

#endif
