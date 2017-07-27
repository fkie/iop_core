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

#ifndef OCU_CONTROL_SERVICE_INFO_H_
#define OCU_CONTROL_SERVICE_INFO_H_

#include <jaustoolset/Transport/JausAddress.h>
#include <iop_msgs_fkie/JausAddress.h>
#include <iop_msgs_fkie/OcuServiceInfo.h>
#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>

namespace iop
{

namespace ocu
{
	class ServiceInfo
	{
	public:
		ServiceInfo(SlaveHandlerInterface &handler, JausAddress &own_addr, std::string uri, jUnsignedByte major_version=1, jUnsignedByte minor_version=0);
		~ServiceInfo();
		SlaveHandlerInterface &handler();
		std::string get_uri();
		bool has_component(JausAddress &address);
		JausAddress &get_own_address();
		JausAddress &get_address();
		/** Sets the address of current controlled service */
		void set_address(JausAddress &address);
		/** Adds the address to the list of discovered services for the given uri.
		 * Returns true, if it is a new one. */
		bool add_discovered(JausAddress address);

	protected:
		SlaveHandlerInterface *p_handler;
		JausAddress p_own_address;
		JausAddress p_address;
		std::string p_uri;
		jUnsignedByte p_major_version;
		jUnsignedByte p_minor_version;
		std::set<jUnsignedInteger> p_discovered_addresses;
	};
};

};

#endif
