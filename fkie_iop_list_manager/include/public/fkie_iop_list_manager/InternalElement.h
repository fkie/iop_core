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


#ifndef IOPINTERNALELEMENT_H
#define IOPINTERNALELEMENT_H

#include "JTSStateMachine.h"
#include "Transport/JausTransport.h"
#include "urn_jaus_jss_core_ListManager/Messages/MessageSet.h"
#include "urn_jaus_jss_core_ListManager/InternalEvents/InternalEventsSet.h"

#include <ros/ros.h>

namespace iop
{

class InternalElement {

public:
	InternalElement();
	InternalElement(urn_jaus_jss_core_ListManager::ReportElement &element);
	~InternalElement();
	bool operator==(InternalElement &value);
	bool operator!=(InternalElement &value);
	jUnsignedShortInteger get_uid();
	jUnsignedShortInteger get_uid_next();
	void set_uids(jUnsignedShortInteger previous, jUnsignedShortInteger next);
	urn_jaus_jss_core_ListManager::ReportElement& get_report();
	jUnsignedShortInteger get_message_id();
	bool update(urn_jaus_jss_core_ListManager::SetElement::Body::SetElementSeq::ElementList::ElementRec &new_data);
	static jUnsignedShortInteger message_id_from_data(const unsigned char *data);

protected:
	urn_jaus_jss_core_ListManager::ReportElement element;

//private:
//	InternalElement(InternalElement const& from);
//	const InternalElement& operator=(const InternalElement& from);

};

};

#endif
