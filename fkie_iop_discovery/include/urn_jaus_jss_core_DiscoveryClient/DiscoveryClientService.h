

#ifndef DISCOVERYCLIENTSERVICE_H
#define DISCOVERYCLIENTSERVICE_H

#include "Service.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JConstants.h"
#include "urn_jaus_jss_core_DiscoveryClient/Messages/MessageSet.h"
#include "urn_jaus_jss_core_DiscoveryClient/InternalEvents/InternalEventsSet.h"
#include "Transport/OS.h"


#include "urn_jaus_jss_core_EventsClient/EventsClientService.h"
#include "urn_jaus_jss_core_Transport/TransportService.h"
#include "DiscoveryClient_ReceiveFSM.h"
#include "DiscoveryClient_SendFSM.h"


namespace urn_jaus_jss_core_DiscoveryClient
{

class DllExport DiscoveryClientService : public JTS::Service
{
public:
	DiscoveryClientService();
	virtual ~DiscoveryClientService();
	void init_service(std::shared_ptr<iop::Component> cmp, JTS::JausRouter* jausRouter, JTS::Service* parentService);
	void registerService(std::string serviceuri, unsigned char maxver, unsigned char minver, JausAddress address)
	{
		pDiscoveryClient_ReceiveFSM->registerService(serviceuri, maxver, minver, address);
	}

	urn_jaus_jss_core_EventsClient::EventsClientService* getParent();

	virtual bool processTransitions(JTS::InternalEvent* ie);
	virtual bool defaultTransitions(JTS::InternalEvent* ie);

	// FSMs are public so that children services can access them
	DiscoveryClient_ReceiveFSM* pDiscoveryClient_ReceiveFSM;
	DiscoveryClient_SendFSM* pDiscoveryClient_SendFSM;
	

protected:
	urn_jaus_jss_core_EventsClient::EventsClientService* pParentService;
	virtual void run();

};

}

#endif // DISCOVERYCLIENTSERVICE_H
