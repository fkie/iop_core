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
		virtual void control_allowed(std::string service_uri, JausAddress component, unsigned char authority) = 0;
		virtual void enable_monitoring_only(std::string service_uri, JausAddress component) = 0;
		virtual void access_deactivated(std::string service_uri, JausAddress component) = 0;
		virtual void create_events(std::string service_uri, JausAddress component, bool by_query=false) = 0;
		virtual void cancel_events(std::string service_uri, JausAddress component, bool by_query=false) = 0;
		// ____________________________
		// === must be overridden =====
		// ============================

		virtual ~SlaveHandlerInterface(){}

	protected:
		SlaveHandlerInterface() {}
	};
};

};

#endif
