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


#include "urn_jaus_jss_core_ListManager/ListManager_ReceiveFSM.h"
#include <iop_component_fkie/iop_config.h>



using namespace JTS;

namespace urn_jaus_jss_core_ListManager
{



ListManager_ReceiveFSM::ListManager_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Management::Management_ReceiveFSM* pManagement_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new ListManager_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pManagement_ReceiveFSM = pManagement_ReceiveFSM;
}



ListManager_ReceiveFSM::~ListManager_ReceiveFSM()
{
	delete context;
}

void ListManager_ReceiveFSM::setupNotifications()
{
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Init", ieHandler, "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Init", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Standby", ieHandler, "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Failure", ieHandler, "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Shutdown", ieHandler, "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled_Emergency", ieHandler, "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Standby", ieHandler, "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_Controlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Ready", ieHandler, "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_Controlled_Ready", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled_Failure", ieHandler, "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_Controlled_Failure", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_Controlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	pManagement_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_ListManager_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "Management_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Standby", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Standby", "ListManager_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Init", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Init", "ListManager_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Failure", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Failure", "ListManager_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Shutdown", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Shutdown", "ListManager_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled_Emergency", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled_Emergency", "ListManager_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_NotControlled", "ListManager_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Standby", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Standby", "ListManager_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Ready", "ListManager_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled_Failure", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled_Failure", "ListManager_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready_Controlled", "ListManager_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving_Ready", "ListManager_ReceiveFSM");
	registerNotification("Receiving", pManagement_ReceiveFSM->getHandler(), "InternalStateChange_To_Management_ReceiveFSM_Receiving", "ListManager_ReceiveFSM");

	iop::Config cfg("~ListManager");
}

void ListManager_ReceiveFSM::deleteElementAction(DeleteElement msg)
{
	p_list.delete_element(msg);
}

void ListManager_ReceiveFSM::sendConfirmElementRequestAction(DeleteElement msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	jUnsignedByte request_id = msg.getBody()->getDeleteElementSeq()->getRequestIDRec()->getRequestID();
	ROS_DEBUG_NAMED("ListManager", "delete for request id %d from %s successful", (int)request_id, sender.str().c_str());
	ConfirmElementRequest reply;
	reply.getBody()->getRequestIDRec()->setRequestID(request_id);
	sendJausMessage(reply, sender);
}

void ListManager_ReceiveFSM::sendRejectElementRequestAction(DeleteElement msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	jUnsignedByte request_id = msg.getBody()->getDeleteElementSeq()->getRequestIDRec()->getRequestID();
	ROS_DEBUG_NAMED("ListManager", "delete element for request id %d from %s failed with error: %d (%s)", (int)request_id, sender.str().c_str(), p_list.get_error_code(), p_list.get_error_msg().c_str());
	RejectElementRequest reject;
	reject.getBody()->getRejectElementRec()->setRequestID(request_id);
	reject.getBody()->getRejectElementRec()->setResponseCode(p_list.get_error_code());
	sendJausMessage(reject, sender);
}

void ListManager_ReceiveFSM::sendReportElementAction(QueryElement msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	jUnsignedShortInteger element_uid = msg.getBody()->getQueryElementRec()->getElementUID();
	ROS_DEBUG_NAMED("ListManager", "send element with uid %d to %s", element_uid, sender.str().c_str());
	ReportElement reply = p_list.get_element(element_uid).get_report();
	sendJausMessage(reply, sender);
}

void ListManager_ReceiveFSM::sendReportElementCountAction(QueryElementCount msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ReportElementCount reply;
	reply.getBody()->getElementCountRec()->setElementCount((unsigned short)p_list.size());
	sendJausMessage(reply, sender);
}

void ListManager_ReceiveFSM::sendReportElementListAction(QueryElementList msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ReportElementList reply = p_list.get_element_list();
	ROS_DEBUG_NAMED("ListManager", "send element list with %d elements to %s", reply.getBody()->getElementList()->getNumberOfElements(), sender.str().c_str());
	sendJausMessage(reply, sender);
}

void ListManager_ReceiveFSM::setElementAction(SetElement msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	jUnsignedByte request_id = msg.getBody()->getSetElementSeq()->getRequestIDRec()->getRequestID();
	bool success = false;
	if (p_list.isElementSupported(msg)) {
		if (p_list.isValidElementRequest(msg)) {
			if (p_list.set_element(msg)) {
				success = true;
			}
		}
	}
	if (success) {
		ROS_DEBUG_NAMED("ListManager", "set element for request id %d from %s successful", (int)request_id, sender.str().c_str());
		ConfirmElementRequest reply;
		reply.getBody()->getRequestIDRec()->setRequestID(request_id);
		sendJausMessage(reply, sender);
	} else {
		ROS_DEBUG_NAMED("ListManager", "set element for request id %d from %s failed with error: %d (%s)", (int)request_id, sender.str().c_str(), p_list.get_error_code(), p_list.get_error_msg().c_str());
		RejectElementRequest reject;
		reject.getBody()->getRejectElementRec()->setRequestID(request_id);
		reject.getBody()->getRejectElementRec()->setResponseCode(p_list.get_error_code());
		sendJausMessage(reject, sender);
	}
}



bool ListManager_ReceiveFSM::elementExists(DeleteElement msg)
{
	// delete supports: 0 first element, 65535: all elements
	return p_list.elementExists(msg);
}

bool ListManager_ReceiveFSM::elementExists(QueryElement msg)
{
	return p_list.elementExists(msg);
}

bool ListManager_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

iop::InternalElementList& ListManager_ReceiveFSM::list_manager()
{
	return p_list;
}

};
