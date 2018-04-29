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

#ifndef OCU_EVENT_HANDLER_INTERFACE_H_
#define OCU_EVENT_HANDLER_INTERFACE_H_

#include "Transport/JausAddress.h"

namespace iop
{
	class EventHandlerInterface
	{
		/**
		 * The interface is used by the iop_event_fkie to forward events to clients.
		 * To receive events IOP client components should include this interface.
		 */
	public:
		// ============================
		// === must be overridden =====
		// ____________________________
		virtual void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata) = 0;
		// ____________________________
		// === must be overridden =====
		// ============================
		virtual void confirmed(JausAddress reporter, unsigned short query_msg_id) {}
		virtual void rejected(JausAddress reporter, unsigned short query_msg_id, unsigned char error_code, std::string error_msg) {}

		virtual ~EventHandlerInterface(){}

	protected:
		EventHandlerInterface() {}
	};

};

#endif
