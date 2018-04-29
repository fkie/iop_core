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

#ifndef IOP_LIST_MANAGER_LISTENER_INTERFACE_H_
#define IOP_LIST_MANAGER_LISTENER_INTERFACE_H_

#include "Transport/JausAddress.h"
#include "urn_jaus_jss_core_ListManager/Messages/MessageSet.h"
#include <iop_list_manager_fkie/InternalElement.h>

namespace iop
{
	class ListManagerListenerInterface
	{
		/**
		 * The interface is used by list manager service to inform services about changes.
		 */
	public:
		// ____________________________
		// === must be overridden =====
		// ============================
		virtual void execute_list(std::vector<iop::InternalElement> elements, double speed = std::numeric_limits<double>::quiet_NaN()) = 0;
		virtual void stop_execution() = 0;

		virtual ~ListManagerListenerInterface(){}

	protected:
		ListManagerListenerInterface() {}
	};

};

#endif
