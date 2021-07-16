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

#ifndef OCU_SLAVE_HANDLER_INTERFACE_H_
#define OCU_SLAVE_HANDLER_INTERFACE_H_

#include "Transport/JausAddress.h"
#include "fkie_iop_component/iop_component.hpp"
#include "fkie_iop_component/timer.hpp"

namespace iop
{
namespace ocu
{
	class SlaveHandlerInterface
	{
		/**
		 * The interface is used by the fkie_iop_ocu_slavelib for communication to iop components.
		 * To receive GUI control commands the IOP client components should include this interface.
		 */
	public:
		// ============================
		// === must be overridden =====
		// ____________________________
		virtual void register_events(JausAddress remote_addr, double hz) = 0;
		virtual void unregister_events(JausAddress remote_addr) = 0;
		virtual void send_query(JausAddress remote_addr) = 0;
		virtual void stop_query(JausAddress remote_addr) = 0;
		// ____________________________
		// === must be overridden =====
		// ============================
		virtual void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
		virtual void enable_monitoring_only(std::string service_uri, JausAddress component);
		virtual void access_deactivated(std::string service_uri, JausAddress component);
		virtual void create_events(std::string service_uri, JausAddress component, bool by_query=false);
		virtual void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);

		virtual ~SlaveHandlerInterface();
		bool has_access();
		bool has_remote_addr();
		JausAddress remote_addr() { return p_remote_addr; }
		void set_rate(double hz);
		void set_supported_service(SlaveHandlerInterface &handler, std::string service_uri, jUnsignedByte major_version, jUnsignedByte minor_version);
		void set_event_name(std::string event_name);
		void set_query_before_event(bool enable, double hz=1.0);

	protected:
		std::shared_ptr<iop::Component> cmp;
		rclcpp::Logger logger;
		iop::Timer p_timer;
		std::string p_name;
		std::string p_service_uri;
		JausAddress p_remote_addr;
		bool p_by_query;
		bool p_has_access;
		double p_hz;
		double p_qbe_hz;
		std::string p_event_name;
		bool p_query_before_event;
		SlaveHandlerInterface(std::shared_ptr<iop::Component> cmp, std::string name, double hz=1.0);
		void p_query_callback();

	};
}

}

#endif
