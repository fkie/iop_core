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

#include <algorithm>
#include <map>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/publisher_options.hpp>
#include "iop_component.hpp"


namespace iop
{

    class Config
    {
      public:

        Config(std::shared_ptr<iop::Component> cmp, std::string ns="~");

        /** Wrapper for get parameter with default options **/
        template<typename T>
        void declare_param(
                std::string name,
                const T& default_value,
                bool read_only=true,
                uint8_t ptype=rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
                std::string description="",
                std::string additional_constraints="")
        {
                std::string pname = name;
               	rcl_interfaces::msg::ParameterDescriptor descriptor;
                if (!p_ns.empty()) {
                        pname = p_ns + "." + name;
                }
	        descriptor.name = pname;
	        descriptor.type = ptype;
	        descriptor.description = description;
	        descriptor.read_only = read_only;
	        descriptor.additional_constraints = additional_constraints;
                p_cmp->declare_parameter<T>(pname, default_value, descriptor);
        }

        /** Wrapper for get parameter with default options **/
        template<typename T>
        std::string get_param(std::string param_name, T& param_val, const T& default_val, bool from_private=true)
        {
                std::string got_from;
                std::string with_ns = p_ns + "." + param_name;
                if (p_cmp->get_parameter<T>(with_ns, param_val)) {
                        got_from = p_ns;
                } else if (from_private && p_cmp->get_parameter<T>(param_name, param_val)) {
                        got_from = "~";
                } else {
                        param_val = default_val;
                        got_from = "default";
                }
                return got_from;
        }

        /** Wrapper for get parameter with default options **/
        template<typename T>
        void param(std::string param_name, T& param_val, const T& default_val,
                bool from_private=true,
                std::string unit="")
        {
                std::string got_from = get_param<T>(param_name, param_val, default_val, from_private);
                RCLCPP_INFO(p_cmp->get_logger(), "[%s] ROS param: %s = %s [ns: %s]", p_ns.c_str(), param_name.c_str(), str_val_unit(param_val, unit).c_str(), got_from.c_str());
        }

        /** Wrapper for get parameter with default options **/
        template<typename T>
        void param_vector(std::string param_name, T& param_val, const T& default_val,
                bool from_private=true,
                std::string unit="")
        {
                std::string got_from = get_param<T>(param_name, param_val, default_val, from_private);
                std::string vstr = str_vector(param_val); 
                RCLCPP_INFO(p_cmp->get_logger(), "[%s] ROS param: %s = %s [ns: %s]", p_ns.c_str(), param_name.c_str(), str_val_unit(vstr, unit).c_str(), got_from.c_str());
        }

        /** Wrapper for get parameter with default options **/
        template<typename T>
        void param_named(std::string param_name, T& param_val, const T& default_val,
                std::map<T, std::string> type_map,
                bool from_private=true,
                std::string unit="")
        {
                std::string got_from = get_param<T>(param_name, param_val, default_val, from_private);
                std::string vname = str_from_map(param_val, type_map);
                RCLCPP_INFO(p_cmp->get_logger(), "[%s] ROS param: %s = %s%s[ns: %s]", p_ns.c_str(), param_name.c_str(), str_val_unit(param_val, unit).c_str(), vname.c_str(), got_from.c_str());
        }

        /** Wrapper for create_publisher **/
        template<
        typename MessageT,
        typename AllocatorT = std::allocator<void>,
        typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>>
        std::shared_ptr<PublisherT> create_publisher(
                const std::string& topic_name,
                const rclcpp::QoS& qos,
                const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options = rclcpp::PublisherOptionsWithAllocator<AllocatorT>()
        )
        {
                std::string name = get_topic_name(topic_name, "topic.pub.");
                auto result = p_cmp->create_publisher<MessageT>(name, qos, options);
                RCLCPP_INFO(p_cmp->get_logger(), "[%s] ROS publisher: %s [type: %s]", p_ns.c_str(), result->get_topic_name(), typeid(MessageT).name());
                return result;
        }

        /** Wrapper for create_subscription **/
        template<
                typename MessageT,
                typename CallbackT,
                typename AllocatorT = std::allocator<void>,
                typename CallbackMessageT =
                        typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
                typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>,
                typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
                        CallbackMessageT,
                        AllocatorT
                >
        >
        std::shared_ptr<SubscriptionT> create_subscription(
                const std::string& topic_name,
                const rclcpp::QoS& qos,
                CallbackT&& callback,
                const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options = rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(),
                typename MessageMemoryStrategyT::SharedPtr msg_mem_strat = ( MessageMemoryStrategyT::create_default() )
        )
        {
                std::string name = get_topic_name(topic_name, "topic.sub.");
                auto result = p_cmp->create_subscription<MessageT>(name, qos, std::forward<CallbackT>(callback), options, msg_mem_strat);
                RCLCPP_INFO(p_cmp->get_logger(), "[%s] ROS subscriber: %s [type: %s]", p_ns.c_str(), result->get_topic_name(), typeid(MessageT).name());
                return result;
        }

        /** Wrapper for create_service **/
        template<typename ServiceT, typename CallbackT>
        typename rclcpp::Service<ServiceT>::SharedPtr
        create_service(
                const std::string& service_name,
                CallbackT&& callback,
                const rmw_qos_profile_t& qos_profile = rmw_qos_profile_services_default,
                rclcpp::CallbackGroup::SharedPtr group = nullptr 
        )
        {
                std::string name = get_topic_name(service_name, "topic.svr.");
                auto result = p_cmp->create_service<ServiceT>(name, std::forward<CallbackT>(callback), qos_profile, group);
                RCLCPP_INFO(p_cmp->get_logger(), "[%s] ROS service: %s [type: %s]", p_ns.c_str(), result->get_service_name(), typeid(ServiceT).name());
                return result;
        }


        template<typename T>
        std::string str_from_map(T& param_val, std::map<T, std::string>& type_map)
        {
                std::ostringstream result;
                typename std::map<T, std::string>::const_iterator it = type_map.find(param_val);
                if (it != type_map.end()) {
                        result << " (" << it->second << ") ";
                }
                return result.str();
        }

        template<typename T>
        std::string str_vector(T& param_val)
        {
                std::ostringstream result;
                std::copy(param_val.cbegin(), param_val.cend(), std::ostream_iterator<typename T::value_type>(result, ", "));
                return result.str();
        }

        template<typename T>
        std::string str_val_unit(T& param_val, std::string& unit)
        {
                std::ostringstream result;
                if (std::is_same<T, int8_t>::value) {
                        result << std::ostringstream::dec << param_val;
                } else if (std::is_same<T, uint8_t>::value) {
                        result << std::ostringstream::dec << param_val;
                } else if (std::is_same<T, double>::value) {
                        result.precision(6);
                        result << param_val;
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
    };
}

#endif
