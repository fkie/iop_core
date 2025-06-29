#ifndef TRANSPORTSERVICE_H
#define TRANSPORTSERVICE_H

#include "Service.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JConstants.h"
#include "urn_jaus_jss_core_Transport/Messages/MessageSet.h"
#include "urn_jaus_jss_core_Transport/InternalEvents/InternalEventsSet.h"
#include "Transport/OS.h"


#include "Transport_ReceiveFSM.h"
#include "Transport_SendFSM.h"


namespace urn_jaus_jss_core_Transport
{

class DllExport TransportService : public JTS::Service
{
public:
	TransportService();
	virtual ~TransportService();
	void init_service(std::shared_ptr<iop::Component> cmp, JTS::JausRouter* jausRouter, JTS::Service* parentService);

	Service* getParent();

	virtual bool processTransitions(JTS::InternalEvent* ie);
	virtual bool defaultTransitions(JTS::InternalEvent* ie);

	// FSMs are public so that children services can access them
	Transport_ReceiveFSM* pTransport_ReceiveFSM;
	Transport_SendFSM* pTransport_SendFSM;
	

protected:
	Service* pParentService;
	virtual void run();

};

}

#endif // TRANSPORTSERVICE_H
