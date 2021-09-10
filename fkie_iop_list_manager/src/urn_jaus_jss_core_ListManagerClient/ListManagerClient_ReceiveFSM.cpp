

#include "urn_jaus_jss_core_ListManagerClient/ListManagerClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_component/iop_component.hpp>



using namespace JTS;
using namespace urn_jaus_jss_core_AccessControlClient;

namespace urn_jaus_jss_core_ListManagerClient
{



ListManagerClient_ReceiveFSM::ListManagerClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: p_timer(std::chrono::seconds(3), std::bind(&ListManagerClient_ReceiveFSM::pTimeout, this), false),
  logger(cmp->get_logger().get_child("ListManagerClient"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new ListManagerClient_ReceiveFSMContext(*this);

	this->pManagementClient_ReceiveFSM = pManagementClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_sync_to_remote = false;
	p_request_id = 0;
	p_current_uid = 0;
	p_request_id_delete_element = 0;
	p_request_id_set_element = 0;
	p_current_access_code = 255;
	p_send_requested = false;
}



ListManagerClient_ReceiveFSM::~ListManagerClient_ReceiveFSM()
{
	delete context;
}

void ListManagerClient_ReceiveFSM::setupNotifications()
{
	pManagementClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_ListManagerClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	pManagementClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_ListManagerClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving_Ready", "ListManagerClient_ReceiveFSM");
	registerNotification("Receiving", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving", "ListManagerClient_ReceiveFSM");
}


void ListManagerClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "ListManagerClient");
}

void ListManagerClient_ReceiveFSM::set_remote(JausAddress &address)
{
	lock_type lock(p_mutex);
	if (p_remote.get() == 0) {
		p_remote = address;
		QueryElementCount query;
		sendJausMessage(query, p_remote);
		p_timer.start();
	} else if (p_remote != address) {
		RCLCPP_WARN(logger, "you try to set access to two different list manager: %s and %s! This is not supported. Use two different components", p_remote.str().c_str(), address.str().c_str());
	}
}

void ListManagerClient_ReceiveFSM::release_remote()
{
	lock_type lock(p_mutex);
	p_timer.stop();
	p_remote = JausAddress(0);
	// reset states
	p_sync_to_remote = false;
	p_send_requested = false;
	p_delete_requested = false;
	p_request_id_delete_element = 0;
	p_request_id_set_element = 0;
	p_current_uid = 0;
	p_request_id = 0;
	p_msgs_2_add.clear();
}

void ListManagerClient_ReceiveFSM::handleConfirmElementRequestAction(ConfirmElementRequest msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	int request_id = (int)msg.getBody()->getRequestIDRec()->getRequestID();
	RCLCPP_DEBUG(logger, "list element confirmed by %s: request_id: %d", sender.str().c_str(), request_id);
	if (p_request_id_delete_element == request_id) {
		p_request_id_delete_element = 0;
		p_delete_requested = false;
		// delete request was successful, send SetElement if available.
		if (pReadyToSet()) {
			p_request_id_set_element = pSendCurrentList();
		}
	} else if (p_request_id_set_element == request_id) {
		p_request_id_set_element = 0;
		p_send_requested = false;
		if (p_msgs_2_add.size() > 0) {
			pInformStateCallbacks(true, p_msgs_2_add.size());
		}
	} else {
		// unexpected confirm, wait for next confirm
	}
	QueryElementCount query;
	sendJausMessage(query, p_remote);
}

void ListManagerClient_ReceiveFSM::handleRejectElementRequestAction(RejectElementRequest msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	int request_id = (int)msg.getBody()->getRejectElementRec()->getRequestID();
	int reject_code = (int)msg.getBody()->getRejectElementRec()->getResponseCode();
	RCLCPP_WARN(logger, "list element rejected by %s: request_id: %d, reject code: %d", sender.str().c_str(), request_id, reject_code);
	if (p_request_id_delete_element == request_id) {
		p_request_id_delete_element = 0;
		// delete request failed, inform caller and request element list
		pInformStateCallbacks(false, p_msgs_2_add.size());
		QueryElementList query;
		sendJausMessage(query, p_remote);
	} else if (p_request_id_set_element == request_id) {
		p_request_id_set_element = 0;
		// set request failed, inform caller and request element list
		if (p_msgs_2_add.size() > 0) {
			pInformStateCallbacks(false, p_msgs_2_add.size());
		}
	} else {
		// unexpected reject id, wait for next confirm
	}
}

void ListManagerClient_ReceiveFSM::handleReportElementAction(ReportElement msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void ListManagerClient_ReceiveFSM::handleReportElementCountAction(ReportElementCount msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	unsigned short count = msg.getBody()->getElementCountRec()->getElementCount();
	RCLCPP_DEBUG(logger, "list manger %s reports %d elements", sender.str().c_str(), count);
	if (count > 0 && p_remote.get() != 0) {
		if (!p_sync_to_remote) {
			QueryElementList query;
			sendJausMessage(query, p_remote);
		}
	} else if (!p_sync_to_remote && p_remote.get() != 0) {
		p_sync_to_remote = true;
	}
}

void ListManagerClient_ReceiveFSM::handleReportElementListAction(ReportElementList msg, Receive::Body::ReceiveRec transportData)
{
	lock_type lock(p_mutex);
	if (p_remote.get() != 0) {
		p_remote_uds.clear();
		p_current_uid = 0;
		ReportElementList::Body::ElementList* list = msg.getBody()->getElementList();
		for (unsigned int i = 0; i < list->getNumberOfElements(); i++) {
			unsigned short uid = list->getElement(i)->getElementUID();
			p_remote_uds.push_back(uid);
			if (uid > p_current_uid && i == 0) {
				p_current_uid = uid;
			}
		}
		p_sync_to_remote = true;
		// perform requests after synchronization
		if (pReadyToDelete()) {
			p_request_id_delete_element = pDeleteRemoteList();
		} else if (pReadyToSet()) {
			p_request_id_set_element = pSendCurrentList();
		}
	}
}

void ListManagerClient_ReceiveFSM::push_back(Element &msg, bool send)
{
	lock_type lock(p_mutex);
	if (p_remote.get() != 0) {
		p_msgs_2_add.push_back(msg);
		p_send_requested |= send;
		if (send && pReadyToSet()) {
			pSendCurrentList();
		}
	} else {
		RCLCPP_WARN(logger, "push_back(Element &) called without set_remote(JausAddress). This call will be ignored!");
	}
}

void ListManagerClient_ReceiveFSM::send_list()
{
	p_send_requested = true;
	if (pReadyToSet()) {
		pSendCurrentList();
	}
}

void ListManagerClient_ReceiveFSM::clear_list()
{
	lock_type lock(p_mutex);
	p_msgs_2_add.clear();
}

void ListManagerClient_ReceiveFSM::delete_remote()
{
	p_delete_requested = true;
	if (pReadyToDelete()) {
		pDeleteRemoteList();
	}
}

bool ListManagerClient_ReceiveFSM::pReadyToSet()
{
	if (p_sync_to_remote) {
		if (p_request_id_delete_element == 0) {
			if (p_request_id_set_element == 0) {
				if (p_send_requested && !p_delete_requested) {
					return true;
				}
			}
		}
	}
	return false;
}

bool ListManagerClient_ReceiveFSM::pReadyToDelete()
{
	if (p_sync_to_remote) {
		if (p_request_id_delete_element == 0) {
			if (p_request_id_set_element == 0) {
				if (p_delete_requested) {
					return true;
				}
			}
		}
	}
	return false;
}

unsigned char  ListManagerClient_ReceiveFSM::pSendCurrentList() {
	lock_type lock(p_mutex);
	if (p_remote.get() != 0 && p_msgs_2_add.size() > 0) {
		p_request_id++;
		SetElement query;
		query.getBody()->getSetElementSeq()->getRequestIDRec()->setRequestID(p_request_id);
		SetElement::Body::SetElementSeq::ElementList* list = query.getBody()->getSetElementSeq()->getElementList();
		for (unsigned int i = 0; i < p_msgs_2_add.size(); i++) {
			SetElement::Body::SetElementSeq::ElementList::ElementRec rec = p_msgs_2_add.at(i);
			rec.setPreviousUID(p_current_uid);
			rec.setElementUID(p_current_uid + 1);
			if (i + 1 < p_msgs_2_add.size()) {
				rec.setNextUID(p_current_uid + 2);
			}
			list->addElement(rec);
			p_current_uid++;
		}
		RCLCPP_DEBUG(logger, "send request %d to add %lu elements @ %s", (int)p_request_id, p_msgs_2_add.size(), p_remote.str().c_str());
		sendJausMessage(query, p_remote);
		return p_request_id;
	}
	return 0;
}

unsigned char ListManagerClient_ReceiveFSM::pDeleteRemoteList()
{
	lock_type lock(p_mutex);
	p_send_requested = false;
	if (p_remote.get() != 0) {
		p_request_id++;
		DeleteElement query;
		query.getBody()->getDeleteElementSeq()->getRequestIDRec()->setRequestID(p_request_id);
		if (p_remote_uds.size() > 0) {
			std::vector<jUnsignedShortInteger>::iterator it;
			for (it = p_remote_uds.begin(); it != p_remote_uds.end(); ++it) {
				DeleteElement::Body::DeleteElementSeq::DeleteElementList::DeleteElementRec item;
				unsigned short uid = *it;
				item.setElementUID(uid);
				query.getBody()->getDeleteElementSeq()->getDeleteElementList()->addElement(item);
			}
		} else {
			DeleteElement::Body::DeleteElementSeq::DeleteElementList::DeleteElementRec item;
			item.setElementUID(65535);
			query.getBody()->getDeleteElementSeq()->getDeleteElementList()->addElement(item);
		}
		RCLCPP_DEBUG(logger, "send request %d to delete %lu elements @ %s", (int)p_request_id, p_remote_uds.size(), p_remote.str().c_str());
		sendJausMessage(query, p_remote);
		return p_request_id;
	} else {
		RCLCPP_WARN(logger, "clear() called without set_remote(JausAddress). This call will be ignored!");
	}
	return 0;
}

void ListManagerClient_ReceiveFSM::pInformStateCallbacks(bool success, unsigned int count)
{
	std::vector<std::function<void (bool success, unsigned int count)> >::iterator it_all;
	for (it_all = p_state_handler.begin(); it_all != p_state_handler.end(); ++it_all) {
		(*it_all)(success, count);
	}
}

void ListManagerClient_ReceiveFSM::pTimeout()
{
	if (!p_sync_to_remote) {
		QueryElementCount query;
		sendJausMessage(query, p_remote);
	} else {
		// this is also a timeout, repeat requests
		if (p_request_id_delete_element > 0) {
			RCLCPP_WARN(logger, "timeout while delete list elements, repeat...");
		}
		if (p_request_id_set_element > 0) {
			RCLCPP_WARN(logger, "timeout while set list elements, repeat...");
		}
		p_request_id_delete_element = 0;
		p_request_id_set_element = 0;
		if (pReadyToDelete()) {
			p_request_id_delete_element = pDeleteRemoteList();
		} else if (pReadyToSet()) {
			p_request_id_set_element = pSendCurrentList();
		}
	}
}


}
