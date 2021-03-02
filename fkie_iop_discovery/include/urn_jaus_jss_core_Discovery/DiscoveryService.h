

#ifndef DISCOVERYSERVICE_H
#define DISCOVERYSERVICE_H

#include "Service.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JConstants.h"
#include "urn_jaus_jss_core_Discovery/Messages/MessageSet.h"
#include "urn_jaus_jss_core_Discovery/InternalEvents/InternalEventsSet.h"
#include "Transport/OS.h"


#include "urn_jaus_jss_core_Events/EventsService.h"
#include "urn_jaus_jss_core_Transport/TransportService.h"
#include "Discovery_ReceiveFSM.h"
#include "Discovery_SendFSM.h"


namespace urn_jaus_jss_core_Discovery
{

class DllExport DiscoveryService : public JTS::Service
{
public:
	DiscoveryService();
	virtual ~DiscoveryService();
	void init_service(std::shared_ptr<iop::Component> cmp, JTS::JausRouter* jausRouter, JTS::Service* parentService);
	void registerService(std::string serviceuri, unsigned char maxver, unsigned char minver, JausAddress address)
	{
		pDiscovery_ReceiveFSM->registerService(serviceuri, maxver, minver, address);
	}

	urn_jaus_jss_core_Events::EventsService* getParent();

	virtual bool processTransitions(JTS::InternalEvent* ie);
	virtual bool defaultTransitions(JTS::InternalEvent* ie);

	// FSMs are public so that children services can access them
	Discovery_ReceiveFSM* pDiscovery_ReceiveFSM;
	Discovery_SendFSM* pDiscovery_SendFSM;
	

protected:
	urn_jaus_jss_core_Events::EventsService* pParentService;
	virtual void run();

};

}

#endif // DISCOVERYSERVICE_H
