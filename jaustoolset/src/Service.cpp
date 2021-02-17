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

#include "Service.h"

namespace JTS
{

//Service::Service(Router *msgRouter)
Service::Service()
{
	m_URN = "Unknown";
}


Service::~Service()
{
}

bool Service::isInitialized()
{
	return p_initialized;
}

const std::string Service::getURN() const
{
	return m_URN;
}

const std::string Service::getName() const
{
	return m_name;
}

unsigned char Service::getVersionManjor() const
{
	return m_version_manjor;
}

unsigned char Service::getVersionMinor() const
{
	return m_version_minor;
}

const std::string Service::getURNInheritsFrom() const
{
	return m_uri_inherits_from;
}

const std::string Service::getNameInheritsFrom() const
{
	return m_name_inherits_from;
}

unsigned char Service::getVersionManjorInheritsFrom() const
{
	return m_inherits_from_version_manjor;
}

unsigned char Service::getVersionMinorInheritsFrom() const
{
	return m_inherits_from_min_version_minor;
}

const std::set<jUnsignedShortInteger> &Service::getInputMessageList() const
{
	return m_InputMessageList;
}


const std::set<jUnsignedShortInteger> &Service::getOutputMessageList() const
{
	return m_OutputMessageList;
}


void Service::processInternalEvent(InternalEvent *ie)
{
    // Invoke the FSM transition for this event.  If no explicit transition
	// is defined, try a default transition...
    if (processTransitions(ie) == false)
    {
        defaultTransitions(ie);
    }
}

}
