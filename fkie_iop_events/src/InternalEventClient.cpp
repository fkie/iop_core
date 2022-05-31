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

#include <algorithm>

#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include <fkie_iop_events/InternalEventClient.h>
#include <fkie_iop_component/iop_component.hpp>

using namespace iop;
using namespace urn_jaus_jss_core_EventsClient;


InternalEventClient::InternalEventClient(rclcpp::Logger& logger, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM& parent, iop::EventHandlerInterface& handler, jUnsignedByte request_id, JTS::Message &query_msg, JausAddress address, jUnsignedByte event_type, double rate)
: logger(logger),
  p_timer(std::chrono::seconds(60), std::bind(&InternalEventClient::timeout, this), false)
{
	p_parent = &parent;
	p_query_msg = &query_msg;
	p_timeout = 0.1;
	p_request_id = request_id;
	p_event_id = 255;
	p_query_msg_id = query_msg.getID();
	p_remote = address;
	p_event_type = event_type;
	p_event_rate = rate;
	p_error_code = 0;
	p_error_msg = "";
	p_wait_for_cancel = false;
	p_canceled = false;
	p_timeout_received = false;
	p_handler.push_back(&handler);

	p_send_create_event();
}

InternalEventClient::~InternalEventClient()
{
	p_timer_stop();
	p_handler.clear();
}

bool InternalEventClient::operator==(InternalEventClient &value)
{
	return (p_query_msg_id == value.p_query_msg_id && p_remote == value.p_remote);
}

bool InternalEventClient::operator!=(InternalEventClient &value)
{
	return !(*this == value);
}

void InternalEventClient::add_handler(iop::EventHandlerInterface& handler)
{
	if (std::find(p_handler.begin(), p_handler.end(), &handler) == p_handler.end()) {
		p_handler.push_back(&handler);
	}
}

void InternalEventClient::remove_handler(iop::EventHandlerInterface& handler) {
	std::vector<iop::EventHandlerInterface *>::iterator it;
	it = std::find(p_handler.begin(), p_handler.end(), &handler);
	if (it != p_handler.end()) {
		p_handler.erase(it);
	}
}

void InternalEventClient::cancel_event(iop::EventHandlerInterface &handler)
{
	remove_handler(handler);
	if (p_handler.size() == 0) {
		p_send_cancel_event();
	}
}

void InternalEventClient::set_timeout(urn_jaus_jss_core_EventsClient::ReportEventTimeout &msg, JausAddress &reporter)
{
	if (reporter.match(p_remote)) {
		p_timeout_received = true;
		double timeout = double(msg.getBody()->getReportTimoutRec()->getTimeout());
		RCLCPP_DEBUG(logger, "update timeout %.2f min for event %d with query=%#x to %s, request_id: %d", timeout, p_event_id, p_query_msg_id, p_remote.str().c_str(), p_request_id);
		if (timeout != p_timeout) {
			if (p_timer.is_running()) {
				p_timer.stop();
				p_send_update_event();
			}
			p_timeout = timeout;
			p_timer.set_interval(std::chrono::milliseconds(int(p_timeout * 60 - 2) / 2 * 1000));
			p_timer_start();
		}
	}

}

bool InternalEventClient::handle_event(urn_jaus_jss_core_EventsClient::Event &msg, JausAddress &reporter)
{
	if (reporter.match(p_remote) && p_event_id == msg.getBody()->getEventRec()->getEventID()) {
		RCLCPP_DEBUG(logger, "received event %d for query=%#x from %s", p_event_id, p_query_msg_id, p_remote.str().c_str());
		for (unsigned int i = 0; i < p_handler.size(); i++) {
			p_handler[i]->event(reporter, p_query_msg_id, msg.getBody()->getEventRec()->getReportMessage()->getLength(), msg.getBody()->getEventRec()->getReportMessage()->getData());
		}
		return true;
	}
	return false;
}

bool InternalEventClient::handle_confirm(urn_jaus_jss_core_EventsClient::ConfirmEventRequest &msg, JausAddress &reporter)
{
	if (reporter.match(p_remote) && p_request_id == msg.getBody()->getConfirmEventRequestRec()->getRequestID()) {
		p_event_id = msg.getBody()->getConfirmEventRequestRec()->getEventID();
		RCLCPP_DEBUG(logger, "event %d confirmed with query=%#x to %s, request_id: %d", p_event_id, p_query_msg_id, p_remote.str().c_str(), p_request_id);
		if (p_wait_for_cancel) {
			p_canceled = true;
		} else if (!p_canceled) {
			p_timer_start();
			for (unsigned int i = 0; i < p_handler.size(); i++) {
				p_handler[i]->confirmed(reporter, p_query_msg_id);
			}
		}
		return true;
	}
	return false;
}

bool InternalEventClient::handle_reject(urn_jaus_jss_core_EventsClient::RejectEventRequest &msg, JausAddress &reporter)
{
	if (reporter.match(p_remote) && p_request_id == msg.getBody()->getRejectEventRequestRec()->getRequestID()) {
		p_error_code = msg.getBody()->getRejectEventRequestRec()->getResponseCode();
		if (msg.getBody()->getRejectEventRequestRec()->isErrorMessageValid()) {
			p_error_msg = std::string(msg.getBody()->getRejectEventRequestRec()->getErrorMessage().c_str());
		}
		RCLCPP_DEBUG(logger, "received reject for event %d with query=%#x to %s, request_id: %d, error: %d, err_msg: %s",
			     p_event_id, p_query_msg_id, p_remote.str().c_str(), p_request_id, p_error_code, p_error_msg.c_str());
		if (p_wait_for_cancel || p_error_code == 5) {
			p_timer_stop();
			for (unsigned int i = 0; i < p_handler.size(); i++) {
				p_handler[i]->rejected(reporter, p_query_msg_id, p_error_code, p_error_msg);
			}
			return true;
		} else {
			p_send_create_event();
			return false;
 		}
	}
	return false;
}


void InternalEventClient::p_send_cancel_event()
{
	if (p_event_id != 255) {
		CancelEvent cancel_event;
		RCLCPP_DEBUG(logger, "Send cancel event %d for query=%#x to %s, request_id: %d", p_event_id, p_query_msg_id, p_remote.str().c_str(), p_request_id);
		cancel_event.getBody()->getCancelEventRec()->setEventID(p_event_id);
		cancel_event.getBody()->getCancelEventRec()->setRequestID(p_request_id);
		p_wait_for_cancel = true;
		p_parent->sendJausMessage(cancel_event, p_remote);
	}
}

void InternalEventClient::set_error(jUnsignedByte code, std::string msg)
{
	p_error_code = code;
	p_error_msg = msg;
}

void InternalEventClient::p_send_update_event()
{
	if (!p_timeout_received && !p_canceled && !p_wait_for_cancel) {
		RCLCPP_DEBUG(logger, "Send get timeout %d for query=%#x to %s, request_id: %d", p_event_id, p_query_msg_id, p_remote.str().c_str(), p_request_id);
		QueryEventTimeout query_timeout;
		p_parent->sendJausMessage(query_timeout, p_remote);
	}

	if (p_event_id != 255 && !p_canceled && !p_wait_for_cancel) {
		RCLCPP_DEBUG(logger, "Send update event %d for query=%#x to %s, request_id: %d", p_event_id, p_query_msg_id, p_remote.str().c_str(), p_request_id);
		jUnsignedInteger len = p_query_msg->getSize();
		unsigned char* bytes = new unsigned char[len];
		p_query_msg->encode(bytes);
		UpdateEvent update_event;
		update_event.getBody()->getUpdateEventRec()->setRequestID(p_request_id);
		update_event.getBody()->getUpdateEventRec()->setEventID(p_event_id);
		update_event.getBody()->getUpdateEventRec()->setEventType(p_event_type);
		update_event.getBody()->getUpdateEventRec()->setRequestedPeriodicRate(p_event_rate);
		update_event.getBody()->getUpdateEventRec()->getQueryMessage()->set(len, bytes);
		p_parent->sendJausMessage(update_event, p_remote);
		delete[] bytes;
	} else {
		p_send_create_event();
	}
}

void InternalEventClient::p_send_create_event() {
	if (p_canceled || p_wait_for_cancel) {
		return;
	}
	if (!p_timeout_received) {
		QueryEventTimeout query_timeout;
		p_parent->sendJausMessage(query_timeout, p_remote);
	}
	RCLCPP_DEBUG(logger, "Send create event of type %d for query=%#x to %s, rate: %f, request_id: %d", (int)p_event_type, p_query_msg->getID(), p_remote.str().c_str(), p_event_rate, p_request_id);
	QueryEventTimeout query_timeout;
	p_parent->sendJausMessage(query_timeout, p_remote);
	jUnsignedInteger len = p_query_msg->getSize();
	unsigned char* bytes = new unsigned char[len];
	p_query_msg->encode(bytes);
	CreateEvent create_event;
	create_event.getBody()->getCreateEventRec()->setRequestID(p_request_id);
	create_event.getBody()->getCreateEventRec()->setEventType(p_event_type);
	create_event.getBody()->getCreateEventRec()->setRequestedPeriodicRate(p_event_rate);
	create_event.getBody()->getCreateEventRec()->getQueryMessage()->set(len, bytes);
	p_parent->sendJausMessage(create_event, p_remote);
	delete[] bytes;
}

void InternalEventClient::timeout()
{
	p_send_update_event();
}

void InternalEventClient::p_timer_stop()
{
	if (p_timer.is_running()) {
		RCLCPP_DEBUG(logger, "stop timeout timer for report %#x with timeout %ldms to %s", p_query_msg_id, p_timer.get_interval().count(), p_remote.str().c_str());
		p_timer.stop();
		p_timeout = 0;
	}
}

void InternalEventClient::p_timer_start()
{
	if (p_timeout > 0) {
		p_timer.stop();
		RCLCPP_INFO(logger, "start timeout timer for %#x with %ldms to %s (component timeout: %.2f min)", p_query_msg_id, p_timer.get_interval().count(), p_remote.str().c_str(), p_timeout);
		p_timer.start();
	}
}
