/*!
 ***********************************************************************
 * @file      JuniorMgr.h
 * @author    Dave Martin, DeVivo AST, Inc.
 * @date      2008/03/03
 *
 *  Copyright (C) 2008. DeVivo AST, Inc
 *
 *  This file is part of Jr Middleware.
 *
 *  Jr Middleware is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Jr Middleware is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with Jr Middleware.  If not, see <http://www.gnu.org/licenses/>.
 *
 ************************************************************************
 */
#include "Transport.h"
#include "JuniorAPI.h"
#include "OS.h"
#include "SimpleThread.h"
#include <stdio.h>
#include <stdlib.h>

namespace DeVivo {
namespace Junior {

typedef std::pair<JAUS_ID, unsigned short> MsgId;
typedef std::pair<unsigned long, MsgId> TimeStampedMsgId;
typedef std::list<TimeStampedMsgId> MsgIdList;
typedef std::list<TimeStampedMsgId>::iterator MsgIdListIter;
const int JrMaxPriority = 15;
const int JrMinPriority = 3;

class JuniorMgr :  public JTS::SimpleThread
{
public:

    JuniorMgr();
    ~JuniorMgr();

    // The public functions mirror the API equivalents.
    JrErrorCode sendto( unsigned int destination, unsigned int size,
                const char* buffer, int priority, int flags, MessageCode code = 0);

    JrErrorCode recvfrom( unsigned int* sender, unsigned int* bufsize,
                  char** buffer, int* priority, int* flags, MessageCode* code = NULL);

    JrErrorCode connect(unsigned int id, std::string config_file);

    unsigned char pending( );
    void stop();

private:

    // Define a couple of private helper functions
    unsigned int umin(unsigned int x, unsigned int y);
    void sendAckMsg(Message* source);
	void sendOrBroadcast(Message* msg);
	bool appendSendMessage(Message* msg);
    bool addMsgToBuffer(Message* msg);
    void checkLargeMsgBuffer();
    bool isDuplicateMsg(Message* msg);
    TimeStampedMsgListIter searchMsgList(TimeStampedMsgList& list,
                                         JAUS_ID sender,
                                         unsigned short seqnum);

    // Private data.
    MessageList        _buffers[JrMaxPriority+1];
    TimeStampedMsgList _largeMsgBuffer;
    JAUS_ID            _id;
    Transport*         _transport;
    unsigned short     _message_counter;
    MsgIdList          _recentMsgs;
    unsigned int       _max_retries;
    unsigned int       _ack_timeout;
    unsigned int       _msg_count;
    unsigned int       _max_msg_size;

    // Configuration data
    unsigned short _maxMsgHistory;      // as a message count
    unsigned short _oldMsgTimeout;      // in seconds
    bool  _detectDuplicates;
    bool  _enableUDPforLocal;

	// We now allow the receive loop to find the ack/nak response,
	// rather than doing it from the send loop.  But that means
	// sharing some data....
	// NOTE: This is likely fragile in a multithreaded environment
	// where two threads are sending on a common Jr handle...
	JAUS_ID _outstanding_ack_request_source;
	unsigned short _outstanding_ack_request_seqnum;
	bool _outstanding_ack_request_acked;

	bool isRunning;
	MessageList        _buffers_send[JrMaxPriority+1];
	DeVivo::Junior::JrSignal _signal_queue_send;
	DeVivo::Junior::JrMutex _lock_queue_send;
	DeVivo::Junior::JrMutex _lock_socket;
	virtual void run();
};

inline TimeStampedMsgListIter JuniorMgr::searchMsgList(
                                         TimeStampedMsgList& list,
                                         JAUS_ID sender,
                                         unsigned short seqnum)
{
    for (TimeStampedMsgListIter iter = list.begin();
         iter != list.end(); iter++)
    {
        if ((iter->second->getSourceId() == sender) &&
            (iter->second->getSequenceNumber() == seqnum))
            return iter;
    }
    return list.end();
}

}} // namespace DeVivo::Junior
