

#include "urn_jaus_jss_core_ListManagerClient/ListManagerClient_ReceiveFSM.h"




using namespace JTS;
using namespace urn_jaus_jss_core_AccessControlClient;

namespace urn_jaus_jss_core_ListManagerClient
{



ListManagerClient_ReceiveFSM::ListManagerClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new ListManagerClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pManagementClient_ReceiveFSM = pManagementClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM->add_reply_handler(&ListManagerClient_ReceiveFSM::access_state, this);
	p_request_id = 0;
	p_current_uid = 0;
	p_request_id_in_process = 0;
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

void ListManagerClient_ReceiveFSM::access_state(JausAddress &address, unsigned char code)
{
	lock_type lock(p_mutex);
	if (code == urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM::ACCESS_STATE_CONTROL_ACCEPTED) {
		if (p_remote != address) {
			p_remote = address;
			QueryElementCount query;
			sendJausMessage(query, p_remote);
		}
	} else {
		p_remote = JausAddress(0);
		// reset states
		p_request_id_in_process = 0;
		p_current_uid = 0;
		p_request_id = 0;
		p_msgs_2_add.clear();
	}
}

void ListManagerClient_ReceiveFSM::handleConfirmElementRequestAction(ConfirmElementRequest msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	int request_id = (int)msg.getBody()->getRequestIDRec()->getRequestID();
	ROS_DEBUG_NAMED("ListManagerClient", "list element confirmed by %s: request_id: %d", sender.str().c_str(), request_id);
	if (p_request_id_in_process == request_id) {
		p_request_id_in_process = 0;
		if (!pSendCurrentList2Remote()) {
			QueryElementList query;
			sendJausMessage(query, p_remote);
		}
	}
}

void ListManagerClient_ReceiveFSM::handleRejectElementRequestAction(RejectElementRequest msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	int request_id = (int)msg.getBody()->getRejectElementRec()->getRequestID();
	int reject_code = (int)msg.getBody()->getRejectElementRec()->getResponseCode();
	ROS_WARN_NAMED("ListManagerClient", "list element rejected by %s: request_id: %d, reject code: %d", sender.str().c_str(), request_id, reject_code);
	if (p_request_id_in_process == request_id) {
		p_request_id_in_process = 255;
		QueryElementList query;
		sendJausMessage(query, p_remote);
		pInformStateCallbacks(false, p_msgs_2_add.size());
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
	ROS_DEBUG_NAMED("ListManagerClient", "list manger %s reports %d elements", sender.str().c_str(), count);
	if (count > 0 && p_remote.get() != 0) {
		QueryElementList query;
		sendJausMessage(query, p_remote);
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
			if (uid > p_current_uid) {
				p_current_uid = uid;
			}
		}
		p_request_id_in_process = 0;
		if (!pSendCurrentList2Remote()) {
			pInformStateCallbacks(true, list->getNumberOfElements());
		}
	}
}

void ListManagerClient_ReceiveFSM::push_back(Element &msg)
{
	lock_type lock(p_mutex);
	if (p_remote.get() != 0) {
		p_msgs_2_add.push_back(msg);
		if (p_request_id_in_process == 0) {
			pSendCurrentList2Remote();
		}
	}
}

void ListManagerClient_ReceiveFSM::clear()
{
	lock_type lock(p_mutex);
	if (p_remote.get() != 0) {
		p_current_uid = 0;
		p_msgs_2_add.clear();
		p_request_id++;
		DeleteElement query;
		query.getBody()->getDeleteElementSeq()->getRequestIDRec()->setRequestID(p_request_id);
		std::vector<jUnsignedShortInteger>::iterator it;
		for (it = p_remote_uds.begin(); it != p_remote_uds.end(); ++it) {
			DeleteElement::Body::DeleteElementSeq::DeleteElementList::DeleteElementRec item;
			unsigned short uid = *it;
			item.setElementUID(uid);
			query.getBody()->getDeleteElementSeq()->getDeleteElementList()->addElement(item);
		}
		ROS_DEBUG_NAMED("ListManagerClient", "send request %d to delete %lu elements @ %s", (int)p_request_id, p_remote_uds.size(), p_remote.str().c_str());
		p_request_id_in_process = p_request_id;
		sendJausMessage(query, p_remote);
	}
}


bool ListManagerClient_ReceiveFSM::pSendCurrentList2Remote() {
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
		ROS_DEBUG_NAMED("ListManagerClient", "send request %d to add %lu elements @ %s", (int)p_request_id, p_msgs_2_add.size(), p_remote.str().c_str());
		p_request_id_in_process = p_request_id;
		sendJausMessage(query, p_remote);
		// added elements are removed from the list. It is not save.
		// If the message goes lost or reject, the elements are lost and will not be resent.
		// TODO: put the to into retry list.
		p_msgs_2_add.clear();
		return true;
	}
	return false;
}

void ListManagerClient_ReceiveFSM::pInformStateCallbacks(bool success, unsigned int count)
{
	std::vector<boost::function<void (bool success, unsigned int count)> >::iterator it_all;
	for (it_all = p_state_handler.begin(); it_all != p_state_handler.end(); ++it_all) {
		(*it_all)(success, count);
	}
}


};
