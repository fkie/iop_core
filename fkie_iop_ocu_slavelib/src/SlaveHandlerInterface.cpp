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

#include "fkie_iop_ocu_slavelib/SlaveHandlerInterface.h"
#include "fkie_iop_ocu_slavelib/Slave.h"

using namespace iop::ocu;

SlaveHandlerInterface::SlaveHandlerInterface(std::shared_ptr<iop::Component> cmp, std::string name, double hz)
: logger(cmp->get_logger().get_child(name)),
  p_timer(std::chrono::milliseconds(static_cast<long>((1.0 / hz) * 1000.0)), std::bind(&SlaveHandlerInterface::p_query_callback, this), false)
{
	this->cmp = cmp;
	p_name = name;
	p_event_name = name;
	p_has_access = false;
	p_by_query = false;
	p_hz = hz;
	p_query_before_event = false;
}

void SlaveHandlerInterface::control_allowed(std::string service_uri, JausAddress component, unsigned char /*authority*/)
{
	if (p_service_uri.compare(service_uri) == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		RCLCPP_WARN(logger, "unexpected control allowed for %s received, ignored!", service_uri.c_str());
	}
}

void SlaveHandlerInterface::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	if (p_service_uri.compare(service_uri) == 0) {
		p_remote_addr = component;
	} else {
		RCLCPP_WARN(logger, "unexpected monitoring for %s received, ignored!", service_uri.c_str());
	}
}

void SlaveHandlerInterface::access_deactivated(std::string service_uri, JausAddress /* component */)
{
	if (p_service_uri.compare(service_uri) == 0) {
		p_has_access = false;
		p_remote_addr = JausAddress(0);
	} else {
		RCLCPP_WARN(logger, "unexpected access deactivation for %s received, ignored!", service_uri.c_str());
	}
}

void SlaveHandlerInterface::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	p_by_query = by_query;
	if (service_uri.empty())
		return;
	if (p_query_before_event) {
		RCLCPP_INFO(logger, "create QUERY timer to get %s from %s", p_event_name.c_str(), component.str().c_str());
		p_timer.set_rate(p_qbe_hz);
		p_timer.start();
		p_query_callback();
		return;
	} else if (by_query) {
		if (p_hz > 0) {
			RCLCPP_INFO(logger, "create QUERY timer to get %s from %s", p_event_name.c_str(), component.str().c_str());
			p_timer.set_rate(p_hz);
			p_timer.start();
		} else {
			RCLCPP_WARN(logger, "invalid hz %.2f for QUERY timer to get %s from %s", p_hz, p_event_name.c_str(), component.str().c_str());
		}
	} else {
		RCLCPP_INFO(logger, "create EVENT to get %s from %s", p_event_name.c_str(), component.str().c_str());
		this->register_events(component, p_hz);
	}
}

void SlaveHandlerInterface::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (service_uri.empty())
		return;
	p_by_query = by_query;
	p_timer.stop();
	if (by_query) {
		this->stop_query(component);
	} else {
		RCLCPP_INFO(logger, "cancel EVENT for %s by %s", p_event_name.c_str(), component.str().c_str());
		this->unregister_events(component);
	}
}

SlaveHandlerInterface::~SlaveHandlerInterface()
{
	p_timer.stop();
}

bool SlaveHandlerInterface::has_access()
{
	return p_has_access;
}

bool SlaveHandlerInterface::has_remote_addr()
{
	return (p_remote_addr.get() != 0);
}

void SlaveHandlerInterface::set_rate(double hz)
{
	p_hz = hz;
}

void SlaveHandlerInterface::set_query_before_event(bool enable, double hz)
{
	if (!enable && p_query_before_event) {
		p_timer.stop();
		if (p_remote_addr.get() != 0) {
			p_query_before_event = enable;
			p_qbe_hz = hz;
			create_events(p_service_uri, p_remote_addr, p_by_query);
		}
	}
	p_query_before_event = enable;
	p_qbe_hz = hz;
}

void SlaveHandlerInterface::set_supported_service(SlaveHandlerInterface &handler, std::string service_uri, jUnsignedByte major_version, jUnsignedByte minor_version)
{
	p_service_uri = service_uri;
	auto slave = Slave::get_instance(cmp);
	slave->add_supported_service(handler, service_uri, major_version, minor_version);
}

void SlaveHandlerInterface::set_event_name(std::string event_name)
{
	p_event_name = event_name;
}

void SlaveHandlerInterface::p_query_callback()
{
	if (p_remote_addr.get() != 0) {
		this->send_query(p_remote_addr);
	}
}

