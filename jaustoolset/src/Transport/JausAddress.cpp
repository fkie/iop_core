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

#include <cstdio>

#include "Transport/JausAddress.h"

JausAddress::JausAddress()
{
	address = malloc(sizeof(jUnsignedInteger));
	if (address != NULL)
	{
		size = sizeof(jUnsignedInteger);
		*((jUnsignedInteger*) address) = 0;
	}
}

JausAddress::JausAddress(jUnsignedShortInteger subsystemID, jUnsignedByte nodeID, jUnsignedByte componentID) : Address()
{
	address = malloc(sizeof(jUnsignedInteger));
	if (address == NULL) return;
	else size = sizeof(jUnsignedInteger);

	jUnsignedInteger tempValue;
	tempValue = (subsystemID << 16) | (nodeID << 8) | componentID;
	*((jUnsignedInteger*)address) = tempValue;
}

JausAddress::JausAddress(jUnsignedInteger value)
{
	address = malloc(sizeof(jUnsignedInteger));
	if (address == NULL) return;
	else size = sizeof(jUnsignedInteger);

	jUnsignedInteger tempValue;
	tempValue = value;
	*((jUnsignedInteger*)address) = tempValue;
}

JausAddress::JausAddress(std::string value)
{
	address = malloc(sizeof(jUnsignedInteger));
	if (address == NULL) return;
	else size = sizeof(jUnsignedInteger);

	int p1, p2, p3;
	int scan_result = std::sscanf(value.c_str(), "%d.%d.%d", &p1, &p2, &p3);
	if (scan_result == 3) {
		jUnsignedInteger tempValue;
		tempValue = (p1 << 16) | (p2 << 8) | p3;
		*((jUnsignedInteger*)address) = tempValue;
	}
}

JausAddress::JausAddress(JausAddress const& from) : Address()
{
	address = malloc(sizeof(jUnsignedInteger));
	if (address == NULL) return;
	else size = sizeof(jUnsignedInteger);

	jUnsignedInteger tempValue;
	tempValue = from.get();
	*((jUnsignedInteger*)address) = tempValue;
}

JausAddress::~JausAddress()
{
	if (address != NULL) free(address);
	size = 0;
}

std::string JausAddress::str() {
	std::ostringstream result;
	result << getSubsystemID() << "." << (int)getNodeID() << "." << (int)getComponentID() << "-" << get();
	return result.str();
}

bool JausAddress::match(JausAddress &expr) {
	bool result = false;
	if (expr.getSubsystemID() != 0 && expr.getSubsystemID() != 65535) {
		if (expr.getSubsystemID() == getSubsystemID()) {
			if (expr.getNodeID() != 0 && expr.getNodeID() != 255) {
				if (expr.getNodeID() == getNodeID()) {
					if (expr.getComponentID() != 0 && expr.getComponentID() != 255) {
						if (expr.getComponentID() == getComponentID()) {
							result = true;
						}
					} else {
						result = true;
					}
				}
			} else {
				result = true;
			}
		}
	} else if (expr.getSubsystemID() == 65535) {
		result = true;
	}
	return result;
}

const JausAddress& JausAddress::operator=(const JausAddress& from)
{
	if (this != &from)
	{
		address = malloc(sizeof(jUnsignedInteger));
		if (address == NULL) return *this;
		else size = sizeof(jUnsignedInteger);

		jUnsignedInteger tempValue;
		tempValue = from.get();
		*((jUnsignedInteger*)address) = tempValue;
	}
	return *this;
}

bool JausAddress::operator==(const JausAddress &value) const
{
	return get() == value.get();
}

bool JausAddress::operator!=(const JausAddress &value) const
{
	return get() != value.get();
}

bool JausAddress::operator<(const JausAddress &value) const
{
	return get() < value.get();
}

jUnsignedInteger JausAddress::get() const
{
	return *((jUnsignedInteger*)address);
}

jUnsignedShortInteger JausAddress::getSubsystemID()
{
	jUnsignedInteger tempValue = *((jUnsignedInteger*)address);
	return (jUnsignedShortInteger)(tempValue >> 16);
}

int JausAddress::setSubsystemID(jUnsignedShortInteger value)
{
	jUnsignedInteger tempValue = *((jUnsignedInteger*)address);
	tempValue = (tempValue | ((jUnsignedInteger)value << 16));
	*((jUnsignedInteger*)address) = tempValue;

	return 0;
}

jUnsignedByte JausAddress::getNodeID()
{
	jUnsignedInteger tempValue = *((jUnsignedInteger*)address);

	tempValue = (tempValue & 0x0000FF00);
	tempValue = tempValue >> 8;

	return (jUnsignedByte)tempValue ;
}

int JausAddress::setNodeID(jUnsignedByte value)
{
	jUnsignedInteger tempValue = *((jUnsignedInteger*)address);
	tempValue = (tempValue | ((jUnsignedInteger)value << 8));
	*((jUnsignedInteger*)address) = tempValue;

	return 0;
}

jUnsignedByte JausAddress::getComponentID()
{
	jUnsignedInteger tempValue = *((jUnsignedInteger*)address);
	return (jUnsignedByte)(tempValue & 0x000000FF);
}

int JausAddress::setComponentID(jUnsignedByte value)
{
	jUnsignedInteger tempValue = *((jUnsignedInteger*)address);
	tempValue = (tempValue | (jUnsignedInteger)value);
	*((jUnsignedInteger*)address) = tempValue;

	return 0;
}

bool JausAddress::isLocalSubsystem(jUnsignedShortInteger sID)
{
	return (getSubsystemID() == sID);
}

bool JausAddress::isLocalSubsystem(JausAddress address)
{
	return (getSubsystemID() == address.getSubsystemID());
}

bool JausAddress::isLocalNode(jUnsignedShortInteger sID, jUnsignedByte nID)
{
	return (isLocalSubsystem(sID) && (getNodeID() == nID));
}

bool JausAddress::isLocalNode(JausAddress address)
{
	return (isLocalSubsystem(address) && (getNodeID() == address.getNodeID()));
}

bool JausAddress::isLocalComponent(jUnsignedShortInteger sID, jUnsignedByte nID, jUnsignedByte cID)
{
	return (isLocalNode(sID, nID) && (getComponentID() == cID));
}

bool JausAddress::isLocalComponent(JausAddress address)
{
	return (isLocalNode(address) && (getComponentID() == address.getComponentID()));
}

