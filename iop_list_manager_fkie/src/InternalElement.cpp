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


#include <ros/ros.h>
#include <ros/console.h>

#include <iop_list_manager_fkie/InternalElement.h>

using namespace iop;

InternalElement::InternalElement()
{
}

InternalElement::InternalElement(urn_jaus_jss_core_ListManager::ReportElement &element)
{
	this->element = element;
}

InternalElement::~InternalElement()
{
}

jUnsignedShortInteger InternalElement::get_uid()
{
	return element.getBody()->getElementRec()->getElementUID();
}

jUnsignedShortInteger InternalElement::get_uid_next()
{
	return element.getBody()->getElementRec()->getNextUID();
}

bool InternalElement::operator==(InternalElement &value)
{
	return get_uid() == value.get_uid();
}

bool InternalElement::operator!=(InternalElement &value)
{
	return !(*this == value);
}

void InternalElement::set_uids(jUnsignedShortInteger previous, jUnsignedShortInteger next)
{
	element.getBody()->getElementRec()->setPreviousUID(previous);
	element.getBody()->getElementRec()->setNextUID(next);
}

urn_jaus_jss_core_ListManager::ReportElement& InternalElement::get_report()
{
	return element;
}

jUnsignedShortInteger InternalElement::get_message_id()
{
	if (element.getBody()->getElementRec()->getElementUID() == 0) {
		return 0;
	}
	return message_id_from_data(element.getBody()->getElementRec()->getElementData()->getData());
}

bool InternalElement::update(urn_jaus_jss_core_ListManager::SetElement::Body::SetElementSeq::ElementList::ElementRec &new_data)
{
	if (get_uid() == 0 || get_uid() == new_data.getElementUID()) {
		element.getBody()->getElementRec()->setElementUID(new_data.getElementUID());
		jUnsignedByte format = new_data.getElementData()->getFormat();
		jUnsignedShortInteger len = new_data.getElementData()->getLength();
		element.getBody()->getElementRec()->getElementData()->set(format, len, (unsigned char *)new_data.getElementData()->getData());
		return true;
	}
	return false;
}

jUnsignedShortInteger InternalElement::message_id_from_data(const unsigned char *data)
{
	jUnsignedShortInteger result = 0;
	try {
		memcpy(&result, data, sizeof(jUnsignedShortInteger));
		result = JSIDL_v_1_0::correctEndianness(result);
	} catch (...) {
	}
	return result;
}
