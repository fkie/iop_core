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


#ifndef IOPINTERNALELEMENTLIST_H
#define IOPINTERNALELEMENTLIST_H

#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_core_ListManager/Messages/MessageSet.h"
#include "urn_jaus_jss_core_ListManager/InternalEvents/InternalEventsSet.h"
#include "InternalElement.h"
#include "ListManagerListenerInterface.h"

#include <boost/thread/recursive_mutex.hpp>
#include <ros/ros.h>
#include <string>

namespace iop
{

class InternalElementList {
public:
	InternalElementList();
	~InternalElementList();

	void register_supported_element(jUnsignedShortInteger report_id, ListManagerListenerInterface *handler);

	/** ========= methods to manage execution ======= **/
	/** On errors False is returned. Check the error_code() and erros_msg() for details. */
	bool execute_list(jUnsignedShortInteger start_uid, double speed = std::numeric_limits<double>::quiet_NaN());
	/** Returns 0 if no valid value is set. */
	jUnsignedShortInteger get_current_element();
	/** Returns 0 if no active elements are available. */
	void set_current_element(jUnsignedShortInteger uid);
	/** The executed service should inform the list if the execution of all his elements is finished.
	 * In this case the list checks if further elements are in the list. Then
	 * the list inform the service about further elements to execute.
	 * Returns false, if no further elements available. */
	bool finished(jUnsignedShortInteger uid, bool execute_next = true);

	/** ========= methods to manage elements ======= **/
	iop::InternalElement get_element(unsigned short uid);
	urn_jaus_jss_core_ListManager::ReportElementList get_element_list();
	/** Adds the elements to the current list.
	 * On errors False is returned. Check the error_code() and erros_msg() for details. */
	bool set_element(urn_jaus_jss_core_ListManager::SetElement msg);
	/** Delete elements specified in given message.
	 * For elements with UID=0 first element in the list will be removed.
	 * For elements with UID=65535 all elements of the list are removed.*/
	bool delete_element(urn_jaus_jss_core_ListManager::DeleteElement msg);
	/** Returns the count of the elements in the list.*/
	unsigned int size();

	/** ========= guard methods ======= **/
	/** On errors False is returned. Check the error_code() and erros_msg() for details. */
	bool elementExists(urn_jaus_jss_core_ListManager::DeleteElement msg);
	/** On errors False is returned. Check the error_code() and erros_msg() for details. */
	bool elementExists(urn_jaus_jss_core_ListManager::QueryElement msg);
	/** register_supported_element() should be called before use this method. Otherwise it will return false. */
	bool isElementSupported(urn_jaus_jss_core_ListManager::SetElement msg);
	/** On errors False is returned. Check the error_code() and erros_msg() for details. */
	bool isValidElementRequest(urn_jaus_jss_core_ListManager::SetElement msg);
	bool elementExists(jUnsignedShortInteger uid);

	/** ========= helper methods ======= **/
	/**
		1: INVALID_UID
		2: INVALID_PREVIOUS
		3: INVALID_NEXT
		4: UNSUPPORTED_TYPE
		5: ELEMENT_NOT_FOUND
		6: OUT_OF_MEMORY
		7: UNSPECIFIED_ERROR
	 */
	jUnsignedByte get_error_code() { return p_error_code; }
	/** Returns message for current error. */
	std::string get_error_msg() { return p_error_msg; }
	void reset_erros();

protected:
	typedef boost::recursive_mutex mutex_type;
	typedef boost::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;

	std::map<jUnsignedShortInteger, ListManagerListenerInterface *> p_supported_elements;
	std::deque<iop::InternalElement> p_element_list;
	jUnsignedByte p_error_code;
	std::string p_error_msg;
	jUnsignedShortInteger p_current_element;
	bool p_loop;

	void set_error(jUnsignedByte code, std::string msg="");
};

};

#endif
