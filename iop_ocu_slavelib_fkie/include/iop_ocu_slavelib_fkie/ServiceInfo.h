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

		static const unsigned char ACCESS_STATE_NOT_AVAILABLE		= 0;
		static const unsigned char ACCESS_STATE_NOT_CONTROLLED		= 1;
		static const unsigned char ACCESS_STATE_CONTROL_RELEASED	= 2;
		static const unsigned char ACCESS_STATE_CONTROL_ACCEPTED	= 3;
		static const unsigned char ACCESS_STATE_TIMEOUT				= 4;
		static const unsigned char ACCESS_STATE_INSUFFICIENT_AUTHORITY	= 5;
		static const unsigned char ACCESS_STATE_MONITORING			= 6;

		static const unsigned char ACCESS_CONTROL_RELEASE = 10;
		static const unsigned char ACCESS_CONTROL_MONITOR = 11;
		static const unsigned char ACCESS_CONTROL_REQUEST = 12;

//		ServiceInfo();
		ServiceInfo(SlaveHandlerInterface &handler, JausAddress &own_addr, std::string uri, jUnsignedByte major_version=1, jUnsignedByte minor_version=0);
		~ServiceInfo();
		SlaveHandlerInterface &handler();
		/** Converts the info of this class the ROS message. */
		iop_msgs_fkie::OcuServiceInfo to_msg();
		/** Returns true if since last call of to_msg() the service info was changed. */
		bool has_changes();

		bool eqaulAddress(iop_msgs_fkie::JausAddress &a1, iop_msgs_fkie::JausAddress &a2);

		bool set_state(unsigned char access_state);
		unsigned char get_state();
		unsigned char get_authority();
		unsigned char get_access_control();

		JausAddress &get_own_address();
		JausAddress &get_address();
		std::string get_uri();
		bool update_cmd(JausAddress &control_addr, unsigned char access_control, unsigned char p_authority);

		/** Adds the address to the list of discovered services for the given uri.
		 * Returns true, if it is a new one. */
		bool add_discovered(JausAddress address);
		/** Returns discovered address for given control_addr. In the component id of
		 * the control address is not 0, the control_addr will be returned. */
		JausAddress get_component_addr(JausAddress &control_addr);

	protected:
		SlaveHandlerInterface *p_handler;
		JausAddress p_own_address;
		std::string p_uri;
		jUnsignedByte p_major_version;
		jUnsignedByte p_minor_version;
		JausAddress p_address;
		std::set<jUnsignedInteger> p_discovered_addresses;
		bool p_do_resume;
		bool p_controled;
		unsigned char p_access_state;
		unsigned char p_access_control;
		unsigned char p_authority;
		bool p_info_updated;


		/** Returns available address for current service.
		 * :param control_addr: can contain only subsystem id or complete address. */
		JausAddress pGetServiceAddress(JausAddress &control_addr);
	};
};

};

#endif
