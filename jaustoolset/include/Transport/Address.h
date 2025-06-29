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

#ifndef ADDRESS_H
#define ADDRESS_H

#include "JausUtils.h"
#include "Transport/OS.h"


class DllExport Address
{
public:
	Address();
	virtual ~Address();
	bool operator==(const Address &value) const;
	bool operator!=(const Address &value) const;
	Address(Address const& from);
	const Address& operator=(const Address& from);


protected:
	void* address;
	int   size;
};


//class JausAddress : public Address
//{
//public:
//	JausAddress();
//	JausAddress(jUnsignedShortInteger subsystemID, jUnsignedByte nodeID, jUnsignedByte componentID);
//	JausAddress(Address value);
//	virtual ~JausAddress();
//	
//	virtual jUnsignedShortInteger getSubsystemID();
//	virtual int setSubsystemID(jUnsignedShortInteger value);
//	virtual jUnsignedByte getNodeID();
//	virtual int setNodeID(jUnsignedByte value);
//	virtual jUnsignedByte getComponentID();
//	virtual int setComponentID(jUnsignedByte value);
//
//	virtual bool isLocalSubsystem(jUnsignedShortInteger sID);
//	virtual bool isLocalSubsystem(JausAddress address);
//	virtual bool isLocalNode(jUnsignedShortInteger sID, jUnsignedByte nID);
//	virtual bool isLocalNode(JausAddress address); 
//	virtual bool isLocalComponent(jUnsignedShortInteger sID, jUnsignedByte nID, jUnsignedByte cID);
//	virtual bool isLocalComponent(JausAddress address);
//};
//
//
//class AS5669AAddress : public Address
//{
//public:
//	AS5669AAddress();
//	AS5669AAddress(jUnsignedInteger id);
//	AS5669AAddress(Address value);
//	virtual ~AS5669AAddress();
//	
//	virtual jUnsignedInteger getID();
//};
// 


#endif // ADDRESS_H

