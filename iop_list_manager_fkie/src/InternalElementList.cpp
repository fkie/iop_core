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


#include <iop_list_manager_fkie/InternalElementList.h>
#include <ros/ros.h>
#include <ros/console.h>

using namespace iop;
using namespace urn_jaus_jss_core_ListManager;


InternalElementList::InternalElementList()
{
	p_error_code = 0;
	p_error_msg = "";
	p_current_element = 0;
	p_loop = false;
}

InternalElementList::~InternalElementList()
{
	lock_type lock(p_mutex);
	p_supported_elements.clear();
	p_element_list.clear();
}

void InternalElementList::register_supported_element(jUnsignedShortInteger report_id, ListManagerListenerInterface *handler)
{
	if (handler != NULL) {
		lock_type lock(p_mutex);
		if (p_supported_elements.find(report_id) == p_supported_elements.end()) {
			ROS_DEBUG_NAMED("InternalElementList", "register supported list report_id: %#x", report_id);
			p_supported_elements[report_id] = handler;
		} else {
			ROS_WARN_NAMED("InternalElementList", "supported list report_id: %#x already registered", report_id);
		}
	} else {
		ROS_WARN_NAMED("InternalElementList", "register supported list report_id %#x failed, because the handler is NULL", report_id);
	}
}

bool InternalElementList::execute_list(jUnsignedShortInteger start_uid, double speed)
{
	lock_type lock(p_mutex);
	reset_erros();
	if (p_current_element != 0) {
		// inform service with current execution to stop.
		p_supported_elements[get_element(p_current_element).get_message_id()]->stop_execution();
		finished(p_current_element, false);
	}
	std::vector<iop::InternalElement> result;
	iop::InternalElement el;
	el = get_element(start_uid);
	if (start_uid == 0) {
		start_uid = el.get_uid();
	}
	if (el.get_uid() == 0) {
		set_error(5, "element " + boost::lexical_cast<std::string>(start_uid) + " not found");
		return false;
	}
	jUnsignedShortInteger msg_id = el.get_message_id();
	ListManagerListenerInterface *handler = p_supported_elements[msg_id];
	if (handler == NULL) {
		set_error(7, "no handler for element " + boost::lexical_cast<std::string>(start_uid) + " specified");
		return false;
	}
	while (el.get_message_id() == msg_id) {
		result.push_back(el);
		el = get_element(el.get_uid_next());
		if (el.get_uid() == start_uid) {
			// we have a loop -> stop fill the list. The execution can be continue after this part of list is executed.
			break;
		} else if (el.get_uid_next() == 0) {
			result.push_back(el);
			break;
		}
	}
	handler->execute_list(result, speed);
	return true;
}


jUnsignedShortInteger InternalElementList::get_current_element()
{
	return p_current_element;
}

void InternalElementList::set_current_element(jUnsignedShortInteger uid)
{
	lock_type lock(p_mutex);
	if (get_element(uid).get_uid() != 0) {
		p_current_element = uid;
	} else {
		p_current_element = 0;
	}
}

bool InternalElementList::finished(jUnsignedShortInteger uid, bool execute_next)
{
	lock_type lock(p_mutex);
	reset_erros();
	std::vector<iop::InternalElement> result;
	iop::InternalElement el = get_element(uid);
	jUnsignedShortInteger msg_id = el.get_message_id();
	if (msg_id == 0) {
		set_error(5, "element " + boost::lexical_cast<std::string>(uid) + " not found");
		return false;
	}
	ListManagerListenerInterface *handler = p_supported_elements[msg_id];
	if (handler == NULL) {
		set_error(7, "no handler for element " + boost::lexical_cast<std::string>(uid) + " specified");
		return false;
	}
	if (execute_next && el.get_uid_next() != 0) {
		if (!execute_list(el.get_uid_next())) {
			return false;
		}
		p_current_element = el.get_uid_next();
	} else {
		p_current_element = 0;
		return false;
	}
	return true;
}

iop::InternalElement InternalElementList::get_element(unsigned short uid)
{
	lock_type lock(p_mutex);
	iop::InternalElement result;
	result.get_report().getBody()->getElementRec()->setElementUID(0);
	jUnsignedShortInteger uid_previous = 0;
	jUnsignedShortInteger uid_next = 0;
	bool found = false;
	for (std::deque<iop::InternalElement>::iterator it = p_element_list.begin(); it != p_element_list.end(); it++) {
		if (found) {
			uid_next = it->get_uid();
			break;
		}
		if (it->get_uid() == uid || uid == 0) {
			result = *it;
			found = true;
			if (p_loop) {
				uid_next = p_element_list.begin()->get_uid();
				break;
			}
		} else {
			uid_previous = it->get_uid();
		}
	}
	result.set_uids(uid_previous, uid_next);
	return result;
}

ReportElementList InternalElementList::get_element_list()
{
	ReportElementList result;
	for (std::deque<iop::InternalElement>::iterator it = p_element_list.begin(); it != p_element_list.end(); it++) {
		ReportElementList::Body::ElementList::ElementListRec rec;
		rec.setElementUID(it->get_uid());
		result.getBody()->getElementList()->addElement(rec);
	}
	return result;
}

bool InternalElementList::set_element(SetElement msg)
{
	lock_type lock(p_mutex);
	reset_erros();
	SetElement::Body::SetElementSeq::ElementList* ellist = msg.getBody()->getSetElementSeq()->getElementList();
	for (unsigned int i = 0; i < ellist->getNumberOfElements(); ++i) {
		SetElement::Body::SetElementSeq::ElementList::ElementRec *elrec = ellist->getElement(i);
		// try to update existing element
		bool found = false;
		for (std::deque<iop::InternalElement>::iterator it = p_element_list.begin(); it != p_element_list.end(); it++) {
			if (it->get_uid() == elrec->getElementUID()) {
				it->update(*elrec);
				found = true;
				break;
			}
		}
		if (!found) {
			if (p_element_list.size() > 65532) {
				set_error(6, "there are already 65533 elements in the list");
				return false;
			}
			// try to find the insert position
			iop::InternalElement ie;
			ie.update(*elrec);
			if (elrec->getPreviousUID() == 0) {
				// we have to insert at the begin
				if (p_element_list.size() > 0 && elrec->getNextUID() != 65535 && elrec->getNextUID() != p_element_list.at(0).get_uid()) {
					set_error(3, "invalid next UID of element at index " + boost::lexical_cast<std::string>(i));
					return false;
				} else {
					p_element_list.insert(p_element_list.begin(), ie);
				}
			} else if (elrec->getNextUID() == 0) {
				// we have to insert at the end
				if (p_element_list.size() > 0 && elrec->getPreviousUID() != 65535 && elrec->getPreviousUID() != p_element_list.at(p_element_list.size() - 1).get_uid()) {
					set_error(3, "invalid previous UID of element at index " + boost::lexical_cast<std::string>(i));
					return false;
				} else {
					p_element_list.push_back(ie);
				}
			} else {
				// find the position of the previous UID
				std::deque<iop::InternalElement>::iterator it;
				bool found = false;
				for (it = p_element_list.begin(); it != p_element_list.end(); it++) {
					if (elrec->getPreviousUID() == 65535) {
						if (it->get_uid() == elrec->getNextUID()) {
							found = true;
							break;
						}
					} else if (it->get_uid() == elrec->getPreviousUID()) {
						found = true;
						it++;
						break;
					}
				}
				if (found) {
					p_element_list.insert(it, ie);
				} else {
					set_error(3, "invalid previous or next UID of element at index " + boost::lexical_cast<std::string>(i));
					return false;
				}
			}
		}
		p_loop = (elrec->getNextUID() == p_element_list.begin()->get_uid());
	}
	return true;
}

bool InternalElementList::delete_element(DeleteElement msg)
{
	lock_type lock(p_mutex);
	DeleteElement::Body::DeleteElementSeq::DeleteElementList* dellist = msg.getBody()->getDeleteElementSeq()->getDeleteElementList();
	for (unsigned int i = 0; i < dellist->getNumberOfElements(); ++i) {
		DeleteElement::Body::DeleteElementSeq::DeleteElementList::DeleteElementRec *delrec = dellist->getElement(i);
		if (delrec->getElementUID() == 0) {
			if (p_element_list.begin() != p_element_list.end()) {
				if (p_current_element == p_element_list.begin()->get_uid()) {
					// stop execution
					p_supported_elements[p_element_list.begin()->get_message_id()]->stop_execution();
					p_current_element = 0;
				}
				ROS_DEBUG_NAMED("InternalElementList", "delete first element with id: %d", p_element_list.begin()->get_uid());
				p_element_list.erase(p_element_list.begin());
			}
		} else if (delrec->getElementUID() == 65535) {
			ROS_DEBUG_NAMED("InternalElementList", "delete all elements");
			if (p_current_element != 0) {
				// stop execution
				iop::InternalElement el = get_element(p_current_element);
				if (el.get_uid() != 0) {
					p_supported_elements[el.get_message_id()]->stop_execution();
				}
				p_current_element = 0;
			}
			p_element_list.clear();
			return true;
		} else {
			jUnsignedShortInteger msg_id = 0;
			for (std::deque<iop::InternalElement>::iterator it = p_element_list.begin(); it != p_element_list.end(); it++) {
				if (it->get_uid() == delrec->getElementUID()) {
					ROS_DEBUG_NAMED("InternalElementList", "delete element with id: %d", it->get_uid());
					if (p_current_element != 0) {
						// stop execution
						if (msg_id != p_element_list.begin()->get_message_id()) {
							msg_id = p_element_list.begin()->get_message_id();
							p_supported_elements[msg_id]->stop_execution();
						}
						p_current_element = 0;
					}
					p_element_list.erase(it);
					break;
				}
			}
		}
	}
	return true;
}

unsigned int InternalElementList::size()
{
	return p_element_list.size();
}

bool InternalElementList::elementExists(DeleteElement msg)
{
	// delete supports: 0 first element, 65535: all elements
	lock_type lock(p_mutex);
	DeleteElement::Body::DeleteElementSeq::DeleteElementList* dellist = msg.getBody()->getDeleteElementSeq()->getDeleteElementList();
	for (unsigned int i = 0; i < dellist->getNumberOfElements(); ++i) {
		DeleteElement::Body::DeleteElementSeq::DeleteElementList::DeleteElementRec *delrec = dellist->getElement(i);
		if (delrec->getElementUID() == 0 || delrec->getElementUID() == 65535) {
			return true;
		} else if (! elementExists(delrec->getElementUID())) {
			return false;
		}
	}
	return true;
}

bool InternalElementList::elementExists(QueryElement msg)
{
	return elementExists(msg.getBody()->getQueryElementRec()->getElementUID());
}

bool InternalElementList::elementExists(jUnsignedShortInteger uid)
{
	lock_type lock(p_mutex);
	reset_erros();
	if (uid == 0 || uid == 65535) {
		set_error(1, "invalid UID");
		return false;
	}
	bool found = false;
	for (std::deque<iop::InternalElement>::iterator it = p_element_list.begin(); it != p_element_list.end(); it++) {
		if (it->get_uid() == uid) {
			found = true;
			break;
		}
	}
	if (!found) {
		set_error(5, "element not found");
		return false;
	}
	return true;
}

bool InternalElementList::isElementSupported(SetElement msg)
{
	lock_type lock(p_mutex);
	reset_erros();
	SetElement::Body::SetElementSeq::ElementList* ellist = msg.getBody()->getSetElementSeq()->getElementList();
	for (unsigned int i = 0; i < ellist->getNumberOfElements(); ++i) {
		SetElement::Body::SetElementSeq::ElementList::ElementRec *elrec = ellist->getElement(i);
		jUnsignedShortInteger msg_uid = InternalElement::message_id_from_data(elrec->getElementData()->getData());
		bool found = false;
		for (std::map<jUnsignedShortInteger, ListManagerListenerInterface *>::iterator it = p_supported_elements.begin(); it != p_supported_elements.end(); it++) {
			if (it->first == msg_uid && it->second != NULL) {
				found = true;
				break;
			}
		}
		if (!found) {
			set_error(4, "encapsulated JAUS message not supported");
			return false;
		}
	}
	return true;
}

bool InternalElementList::isValidElementRequest(SetElement msg)
{
	lock_type lock(p_mutex);
	reset_erros();
	SetElement::Body::SetElementSeq::ElementList* ellist = msg.getBody()->getSetElementSeq()->getElementList();
	std::set<jUnsignedShortInteger> new_element_ids;
	if (ellist->getNumberOfElements() == 0) {
		set_error(5, "no element found");
		return false;
	}
	for (unsigned int i = 0; i < ellist->getNumberOfElements(); ++i) {
		SetElement::Body::SetElementSeq::ElementList::ElementRec *elrec = ellist->getElement(i);
		if (elrec->getElementUID() == 0 || elrec->getElementUID() == 65535) {
			set_error(1, "invalid UID: " + boost::lexical_cast<std::string>(elrec->getElementUID()));
			return false;
		}
		new_element_ids.insert(elrec->getElementUID());
		if (elrec->getPreviousUID() != 0 && elrec->getPreviousUID() != 65535) {
			if (new_element_ids.find(elrec->getPreviousUID()) == new_element_ids.end()) {
				if (!elementExists(elrec->getPreviousUID())) {
					set_error(2, "invalid previous UID: " + boost::lexical_cast<std::string>(elrec->getPreviousUID()));
					return false;
				}
			}
		}
	}
	// test for next uid
	for (unsigned int i = 0; i < ellist->getNumberOfElements(); ++i) {
		SetElement::Body::SetElementSeq::ElementList::ElementRec *elrec = ellist->getElement(i);
		if (elrec->getNextUID() != 0 && elrec->getNextUID() != 65535) {
			if (new_element_ids.find(elrec->getNextUID()) == new_element_ids.end()) {
				if (!elementExists(elrec->getNextUID())) {
					set_error(3, "invalid next UID: " + boost::lexical_cast<std::string>(elrec->getNextUID()));
					return false;
				}
			}
		}
	}
	return true;
}


void InternalElementList::set_error(jUnsignedByte code, std::string msg)
{
	p_error_code = code;
	p_error_msg = msg;
}

void InternalElementList::reset_erros()
{
	set_error(0);
}
