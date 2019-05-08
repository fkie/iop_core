/***********           LICENSE HEADER   *******************************
JAUS Tool Set
Copyright (c)  2010, United States Government
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

       Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

       Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

       Neither the name of the United States Government nor the names of
its contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*********************  END OF LICENSE ***********************************/

#include <iostream>
#include "Transport/JausTransport.h"
#include "Transport/JuniorAPI.h"

namespace JTS
{

/**
 * JausRouter Definitions
 */
JausRouter::JausRouter(JausAddress jausAddress, InternalEventHandler* ieHandler, std::string config)
{
	this->jausAddress = jausAddress;
	jrHandle = 0;
	isRunning = false;
	int error = JrConnect(jausAddress.get(), config.c_str(), &jrHandle);
	if (error != 0) {
		pIsConnected = false;
	} else {
		pIsConnected = true;
	}
	this->ieHandler = ieHandler;
}

JausRouter::~JausRouter()
{
	JrDisconnect(jrHandle);
}

void JausRouter::routeMessage(JausAddress sender, unsigned int bufsize, const unsigned char* buffer)
{
#ifdef DEBUG
	unsigned short msg_id = *((unsigned short*) buffer);
	std::cout << "[JausRouter::routeMessage] Local Component : 0x" << std::hex << msg_id << std::dec << std::endl;
#endif

        Receive* message = new Receive();
        message->getBody()->getReceiveRec()->getSourceID()->setSubsystemID(sender.getSubsystemID());
        message->getBody()->getReceiveRec()->getSourceID()->setNodeID(sender.getNodeID());
        message->getBody()->getReceiveRec()->getSourceID()->setComponentID(sender.getComponentID());
        message->getBody()->getReceiveRec()->getMessagePayload()->set(bufsize, (unsigned char*) buffer);
        ieHandler->invoke(message);
}

void JausRouter::sendMessage(Send* msg, int priority)
{
#ifdef DEBUG
                unsigned short msg_id = *((unsigned short*) msg->getBody()->getSendRec()->getMessagePayload()->getData());
                std::cout << "[JausRouter::sendMessage] Sending msg 0x" << std::hex << msg_id << std::dec << std::endl;
#endif

        /// Pull the destination ID
        JausAddress destination(msg->getBody()->getSendRec()->getDestSubsystemID(),
                                                        msg->getBody()->getSendRec()->getDestNodeID(),
                                                        msg->getBody()->getSendRec()->getDestComponentID());

        // If the destination is local, loopback to the routeMessage function
        if (destination == jausAddress)
        {
                        routeMessage(destination,
                                msg->getBody()->getSendRec()->getMessagePayload()->getLength(),
                                msg->getBody()->getSendRec()->getMessagePayload()->getData());
        }
        // Otherwise, forward Message to NodeManager.
        else
          //JrErrorCode ret =
          JrSend(jrHandle, destination.get(),
                 msg->getBody()->getSendRec()->getMessagePayload()->getLength(),
                 (const char*) msg->getBody()->getSendRec()->getMessagePayload()->getData(),
				 priority);
}


void JausRouter::stop()
{
	runLock.lock();
	isRunning = false;
	runLock.unlock();

	// Send a message to ourselves to wake-up the thread
	unsigned short msg_id = 0;
	JrSend(jrHandle, jausAddress.get(), 2, (char*) &msg_id);
}


JausAddress* const JausRouter::getJausAddress()
{
	return &jausAddress;
}



void JausRouter::run()
{
	char* buffer;
	unsigned int bufsize;
	unsigned int source;
	int priority;
	int flags;
	unsigned short msg_id;

	runLock.lock();
	isRunning = true;
	runLock.unlock();

#ifdef DEBUG
	std::cout << "[JausRouter::run]" << std::endl;
#endif

	while (isRunning)
	{
		/*
		 *  Transport Service receives a TransportMessage
		 */
		if (JrReceive(jrHandle, &source, &bufsize, &buffer, &priority, &flags, &msg_id)  == Ok)
		{
#ifdef DEBUG
	std::cout << "[JausRouter::receive] Message size " << bufsize << " bytes " << std::endl;
#endif

			// See if we got an exit signal while we were pending
			if (!isRunning) break;

			// Create a component message wrapper to pass to the services...
			JausAddress sender(source);
			routeMessage(source, bufsize, (unsigned char*) buffer);
			if (buffer)
			{
				delete[] buffer;
				buffer = NULL;
			}
		}
		else
		{
			DeVivo::Junior::JrSleep(1); // throttle
		}
	}

#ifdef DEBUG
	std::cout << "[JausRouter::shutdown]" << std::endl;
#endif
}



};

