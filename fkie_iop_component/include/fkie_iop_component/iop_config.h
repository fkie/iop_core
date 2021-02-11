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
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/publisher_options.hpp>
#include "fkie_iop_component/ros_node.hpp"


namespace iop
{
//     static void tokenize(std::string const &str, const char delim, std::vector<std::string> &out)
//     {
//         size_t start;
//         size_t end = 0;
        
//         while ((start = str.find_first_not_of(delim, end)) != std::string::npos) {
//                 end = str.find(delim, start);
//                 out.push_back(str.substr(start, end - start));
//         }
//     }


    class Config
    {
      public:

        Config(std::string ns="~");

        /** Wrapper for get parameter with default options **/
        template<typename T>
        void param(std::string param_name, T& param_val, T default_val,
                bool from_private=true,
                std::string unit="",
                std::map<int, std::string> type_map = std::map<int, std::string>()
        );

        /** Wrapper for create_publisher **/
        template<
        typename MessageT,
        typename AllocatorT = std::allocator<void>,
        typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>>
        std::shared_ptr<PublisherT> create_publisher(
                const std::string& topic_name,
                const rclcpp::QoS& qos,
                const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options = rclcpp::PublisherOptionsWithAllocator<AllocatorT>()
        );

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
        );
        // {
        //         std::string name = get_topic_name(topic_name, "topic_sub_");
        //         auto result = p_rosnode->create_subscription(name, qos, std::forward<CallbackT>(callback), options, msg_mem_strat);
        //         RCLCPP_INFO(p_rosnode->get_logger(), "ROS subscriber[%s]: %s <%s>", p_ns.c_str(), result.get_topic_name(), typeid(MessageT).name());
        //         return result;
        // }

        // template<
        //         typename MessageT,
        //         typename CallbackT,
        //         typename AllocatorT = std::allocator<void>,
        //         typename CallbackMessageT =
        //                 typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
        //         typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>,
        //         typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
        //                 CallbackMessageT,
        //                 AllocatorT
        //         >
        // >
        // std::shared_ptr<SubscriptionT> create_subscription(
        //         const std::string & topic_name,
        //         const rclcpp::QoS & qos,
        //         CallbackT && callback,
        //         const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options = rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(),
        //         typename MessageMemoryStrategyT::SharedPtr msg_mem_strat = (MessageMemoryStrategyT::create_default())
        // )
        // {
        //         std::string name = get_topic_name(topic_name, "topic_sub_");
        //         auto result = p_rosnode->create_subscription(name, qos, callback, options, msg_mem_strat);
        //         RCLCPP_INFO(p_rosnode->get_logger(), "ROS subscriber[%s]: %s <%s>", p_ns.c_str(), result.get_topic_name(), typeid(MessageT).name());
        //         return result;
        // }
        /** Wrapper for create_service **/
        template<typename ServiceT, typename CallbackT>
        typename rclcpp::Service<ServiceT>::SharedPtr
        create_service(
                const std::string& service_name,
                CallbackT&& callback,
                const rmw_qos_profile_t& qos_profile = rmw_qos_profile_services_default,
                rclcpp::CallbackGroup::SharedPtr group = nullptr 
        );

        std::string tostr(char& param_val, std::string unit, std::map<int, std::string>& type_map);
        std::string tostr(unsigned char& param_val, std::string unit, std::map<int, std::string>& type_map);
        std::string tostr(int& param_val, std::string unit, std::map<int, std::string>& type_map);
        std::string tostr(double& param_val, std::string unit, std::map<int, std::string>& type_map);
        std::string tostr(float& param_val, std::string unit, std::map<int, std::string>& type_map);
        std::string tostr(bool& param_val, std::string unit, std::map<int, std::string>& type_map);
        std::string tostr(std::string& param_val, std::string unit, std::map<int, std::string>& type_map);
        std::string tostr(std::vector<int>& param_val, std::string unit, std::map<int, std::string>& type_map);
        std::string tostr(std::vector<double>& param_val, std::string unit, std::map<int, std::string>& type_map);
        std::string tostr(std::vector<float>& param_val, std::string unit, std::map<int, std::string>& type_map);
        std::string tostr(std::vector<std::string>& param_val, std::string unit, std::map<int, std::string>& type_map);

      protected:
        std::string p_ns;
        iop::RosNode* p_rosnode;

        std::string get_topic_name(std::string name, std::string prefix);
    };
}

#endif
