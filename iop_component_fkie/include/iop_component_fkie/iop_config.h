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

#include <map>
#include <ros/ros.h>

namespace iop
{
	class Config
	{
	public:

		Config(std::string ns="~")
		{
			p_ns = ns;
			p_pnh = ros::NodeHandle("~");
			p_snh = ros::NodeHandle(ns);
		}

		template<typename T>
		void param(std::string param_name, T &param_val, T default_val,
				bool from_private=true, bool from_ns=true,
				std::string unit="",
				std::map<int, std::string> type_map = std::map<int, std::string>())
		{
			std::string got_from;
			if (p_snh.hasParam(param_name)) {
				p_snh.param(param_name, param_val, default_val);
				got_from = p_ns;
			} else if (from_private && p_pnh.getNamespace().compare(p_snh.getNamespace()) != 0 && p_pnh.hasParam(param_name)) {
				p_pnh.param(param_name, param_val, default_val);
				got_from = "~";
			} else if (from_ns && p_nh.hasParam(param_name)) {
				p_nh.param(param_name, param_val, default_val);
				got_from = p_nh.getNamespace();
			} else {
				p_snh.param(param_name, param_val, default_val);
				got_from = "default";
			}
			ROS_INFO_STREAM("ROS param[" << p_ns << "]: " << param_name << " = " << tostr(param_val, unit, type_map) << " [ns: " << got_from << "]");
		}

		template <class M>
		ros::Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false)
		{
			ros::Publisher result = p_nh.advertise<M>(topic, queue_size, latch);
			ROS_INFO_STREAM("ROS publisher[" << p_ns << "]: " << result.getTopic() << " <" << typeid(M).name()  << " ");
			return result;
		}

		template <class M>
		ros::Publisher advertise(const std::string& topic, uint32_t queue_size,
			const ros::SubscriberStatusCallback& connect_cb,
			const ros::SubscriberStatusCallback& disconnect_cb = ros::SubscriberStatusCallback(),
			const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
			bool latch = false)
		{
			ros::Publisher result = p_pnh.advertise<M>(topic, queue_size, connect_cb, disconnect_cb, tracked_object, latch);
			ROS_INFO_STREAM("ROS publisher[" << p_ns << "]: " << result.getTopic() << " <" << typeid(M).name() << " ");
			return result;
		}

		template <class M>
		ros::Publisher advertise_p(const std::string& topic, uint32_t queue_size, bool latch = false)
		{
			ros::Publisher result = p_pnh.advertise<M>(topic, queue_size, latch);
			ROS_INFO_STREAM("ROS publisher[" << p_ns << "]: " << result.getTopic() << " <" << typeid(M).name() << " ");
			return result;
		}

		template<class T, class MReq, class MRes>
		ros::ServiceServer advertiseService(const std::string& service, bool(T::*srv_func)(MReq &, MRes &), T *obj)
		{
			ros::ServiceServer result = p_pnh.advertiseService(service, srv_func, obj);
			ROS_INFO_STREAM("ROS service[" << p_ns << "]: " << service << " <" << typeid(MReq).name() << "> - <" << typeid(MRes).name() << "");
			return result;
		}

		template<class M, class T>
		ros::Subscriber subscribe(const std::string& topic, uint32_t queue_size,
				void(T::*fp)(const boost::shared_ptr<M const>&), T* obj,
				const ros::TransportHints& transport_hints = ros::TransportHints())
		{
			ros::Subscriber result = p_nh.subscribe(topic, queue_size, fp, obj, transport_hints);
			ROS_INFO_STREAM("ROS subscriber[" << p_ns << "]: " << result.getTopic() << " <" << typeid(M).name() << " ");
			return result;
		}

		template<class M, class T>
		ros::Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj,
				const ros::TransportHints& transport_hints = ros::TransportHints())
		{
			ros::Subscriber result = p_nh.subscribe(topic, queue_size, fp, obj, transport_hints);
			ROS_INFO_STREAM("ROS subscriber[" << p_ns << "]: " << result.getTopic() << " <" << typeid(M).name() << " ");
			return result;
		}

		template<class M, class T>
		ros::Subscriber subscribe_p(const std::string& topic, uint32_t queue_size,
				void(T::*fp)(const boost::shared_ptr<M const>&), T* obj,
				const ros::TransportHints& transport_hints = ros::TransportHints())
		{
			ros::Subscriber result = p_pnh.subscribe(topic, queue_size, fp, obj, transport_hints);
			ROS_INFO_STREAM("ROS subscriber[" << p_ns << "]: " << result.getTopic() << " <" << typeid(M).name() << " ");
			return result;
		}

		std::string tostr(int& param_val, std::string unit, std::map<int, std::string>& type_map)
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

		std::string tostr(double& param_val, std::string unit, std::map<int, std::string>& type_map)
		{
			std::ostringstream result;
			result << param_val;
			if (!unit.empty()) {
				result << " " << unit;
			}
			return result.str();
		}

		std::string tostr(float& param_val, std::string unit, std::map<int, std::string>& type_map)
		{
			std::ostringstream result;
			result << param_val;
			if (!unit.empty()) {
				result << " " << unit;
			}
			return result.str();
		}

		std::string tostr(bool& param_val, std::string unit, std::map<int, std::string>& type_map)
		{
			std::ostringstream result;
			result << param_val;
			if (!unit.empty()) {
				result << " " << unit;
			}
			return result.str();
		}

		std::string tostr(std::string& param_val, std::string unit, std::map<int, std::string>& type_map)
		{
			if (!unit.empty()) {
				return param_val + " " + unit;
			}
			return param_val;
		}

		std::string tostr(std::vector<int>& param_val, std::string unit, std::map<int, std::string>& type_map)
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

		std::string tostr(XmlRpc::XmlRpcValue& param_val, std::string unit, std::map<int, std::string>& type_map)
		{
			std::ostringstream result;
			result << param_val;
			if (!unit.empty()) {
				result << " " << unit;
			}
			return result.str();
		}

	protected:
		std::string p_ns;
		ros::NodeHandle p_nh;
		ros::NodeHandle p_pnh;
		ros::NodeHandle p_snh;

	};
};

#endif
