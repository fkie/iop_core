

#ifndef LISTMANAGERCLIENT_RECEIVEFSM_H
#define LISTMANAGERCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_core_ListManagerClient/Messages/MessageSet.h"
#include "urn_jaus_jss_core_ListManagerClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_ManagementClient/ManagementClient_ReceiveFSM.h"

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include "ListManagerClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_core_ListManagerClient
{

#define Element SetElement::Body::SetElementSeq::ElementList::ElementRec

class DllExport ListManagerClient_ReceiveFSM : public JTS::StateMachine
{
public:
	ListManagerClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM);
	virtual ~ListManagerClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleConfirmElementRequestAction(ConfirmElementRequest msg, Receive::Body::ReceiveRec transportData);
	virtual void handleRejectElementRequestAction(RejectElementRequest msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportElementAction(ReportElement msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportElementCountAction(ReportElementCount msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportElementListAction(ReportElementList msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods

	void set_remote(JausAddress &address);
	void release_remote();

	void push_back(Element &msg, bool send);
	void send_list();
	void clear_list();
	void delete_remote();
	template<class T>
	void add_state_handler(void(T::*handler)(bool success, unsigned int count), T*obj) {
		p_state_handler.push_back(boost::bind(handler, obj, _1, _2));
	}

	ListManagerClient_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM;

	std::vector<boost::function<void (bool success, unsigned int count)> > p_state_handler;
	typedef boost::recursive_mutex mutex_type;
	typedef boost::unique_lock<mutex_type> lock_type;
	mutable mutex_type p_mutex;

	ros::NodeHandle p_nh;
	ros::Timer p_timer;
	JausAddress p_remote;
	std::vector<jUnsignedShortInteger> p_remote_uds;
	std::vector<Element> p_msgs_2_add;

	bool p_sync_to_remote;
	unsigned char p_request_id;
	unsigned char p_request_id_delete_element;
	unsigned char p_request_id_set_element;
	unsigned short p_current_uid;
	unsigned char p_current_access_code;
	bool p_send_requested;
	bool p_delete_requested;

	bool pReadyToSet();
	bool pReadyToDelete();
	unsigned char pSendCurrentList();
	unsigned char pDeleteRemoteList();
	void pInformStateCallbacks(bool success, unsigned int count);

	void p_timeout(const ros::TimerEvent& event);
	void p_timer_stop();
	void p_timer_start();

};

};

#endif // LISTMANAGERCLIENT_RECEIVEFSM_H
