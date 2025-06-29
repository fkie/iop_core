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

#ifndef JTS_SEND_VERSION_H
#define JTS_SEND_VERSION_H

#include "JausUtils.h"
#include "InternalEvents/InternalEvent.h"
#include "Transport/OS.h"
// removed namespace to avoid compiler erro C2872 in visual studio
//namespace JTS
//{

class DllExport Send: public JTS::InternalEvent
{
public:
	class DllExport Body
	{
	public:
		class DllExport SendRec
		{
		public:
			class DllExport DestinationID
			{
			public:
				jUnsignedInteger getComponentID();
				int setComponentID(jUnsignedInteger value);
				jUnsignedInteger getNodeID();
				int setNodeID(jUnsignedInteger value);
				jUnsignedInteger getSubsystemID();
				int setSubsystemID(jUnsignedInteger value);
				unsigned int getSize();
				virtual void encode(unsigned char *bytes);
				virtual void decode(const unsigned char *bytes);
				DestinationID &operator=(const DestinationID &value);
				bool operator==(const DestinationID &value) const;
				bool operator!=(const DestinationID &value) const;
				DestinationID();
				DestinationID(const DestinationID &value);
				virtual ~DestinationID();

			protected:
				jUnsignedInteger m_SubFields;
			};
			class DllExport SourceID
			{
			public:
				jUnsignedInteger getComponentID();
				int setComponentID(jUnsignedInteger value);
				jUnsignedInteger getNodeID();
				int setNodeID(jUnsignedInteger value);
				jUnsignedInteger getSubsystemID();
				int setSubsystemID(jUnsignedInteger value);
				unsigned int getSize();
				virtual void encode(unsigned char *bytes);
				virtual void decode(const unsigned char *bytes);
				SourceID &operator=(const SourceID &value);
				bool operator==(const SourceID &value) const;
				bool operator!=(const SourceID &value) const;
				SourceID();
				SourceID(const SourceID &value);
				virtual ~SourceID();

			protected:
				jUnsignedInteger m_SubFields;
			};
			class DllExport MessagePayload
			{
			public:
				jUnsignedInteger getLength() const;
				const unsigned char *getData() const;
				int set(const jUnsignedInteger &length, const unsigned char *data);
				unsigned int getSize();
				virtual void encode(unsigned char *bytes);
				virtual void decode(const unsigned char *bytes);
				MessagePayload &operator=(const MessagePayload &value);
				bool operator==(const MessagePayload &value) const;
				bool operator!=(const MessagePayload &value) const;
				MessagePayload();
				MessagePayload(const MessagePayload &value);
				virtual ~MessagePayload();

			protected:
				jUnsignedInteger m_Length;
				unsigned char *m_Data;
			};

			jUnsignedByte getPresenceVector();
			bool checkPresenceVector(unsigned int index) const;
			/** --begin -- Receive 1.0 compatibility **/
			jUnsignedShortInteger getDestSubsystemID();
			int setDestSubsystemID(jUnsignedShortInteger value);
			jUnsignedByte getDestNodeID();
			int setDestNodeID(jUnsignedByte value);
			jUnsignedByte getDestComponentID();
			int setDestComponentID(jUnsignedByte value);
			bool isSrcSubsystemIDValid() const;
			jUnsignedShortInteger getSrcSubsystemID();
			int setSrcSubsystemID(jUnsignedShortInteger value);
			bool isSrcNodeIDValid() const;
			jUnsignedByte getSrcNodeID();
			int setSrcNodeID(jUnsignedByte value);
			jUnsignedByte getSrcComponentID();
			int setSrcComponentID(jUnsignedByte value);
			/** --end -- Receive 1.0 compatibility **/
			jUnsignedByte getReliableDelivery();
			int setReliableDelivery(jUnsignedByte value);
			DestinationID* const getDestinationID();
			int setDestinationID(const DestinationID &value);
			bool isSourceIDValid() const;
			SourceID* const getSourceID();
			int setSourceID(const SourceID &value);
			bool isPriorityValid() const;
			jUnsignedByte getPriority();
			int setPriority(jUnsignedByte value);
			MessagePayload* const getMessagePayload();
			int setMessagePayload(const MessagePayload &value);
			unsigned int getSize();
			virtual void encode(unsigned char *bytes);
			virtual void decode(const unsigned char *bytes);
			SendRec &operator=(const SendRec &value);
			bool operator==(const SendRec &value) const;
			bool operator!=(const SendRec &value) const;
			SendRec();
			SendRec(const SendRec &value);
			virtual ~SendRec();

		protected:
			int setPresenceVector(unsigned int index);

			jUnsignedByte m_PresenceVector;
			jUnsignedByte m_ReliableDelivery;
			DestinationID m_DestinationID;
			SourceID m_SourceID;
			jUnsignedByte m_Priority;
			MessagePayload m_MessagePayload;
		};

		SendRec* const getSendRec();
		int setSendRec(const SendRec &value);
		unsigned int getSize();
		virtual void encode(unsigned char *bytes);
		virtual void decode(const unsigned char *bytes);
		Body &operator=(const Body &value);
		bool operator==(const Body &value) const;
		bool operator!=(const Body &value) const;
		Body();
		Body(const Body &value);
		virtual ~Body();

	protected:
		SendRec m_SendRec;
	};

	Body* const getBody();
	int setBody(const Body &value);
	const unsigned int getSize();
	virtual void encode(unsigned char *bytes);
	virtual void decode(const unsigned char *bytes);
	Send &operator=(const Send &value);
	bool operator==(const Send &value) const;
	bool operator!=(const Send &value) const;
	Send();
	Send(const Send &value);
	virtual ~Send();

protected:
	Body m_Body;
};

//}

#endif
