%copyright%

#ifndef %service_name_allcaps%_H
#define %service_name_allcaps%_H

#include "Service.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JConstants.h"
#include "%service_namespace%/Messages/MessageSet.h"
#include "%service_namespace%/InternalEvents/InternalEventsSet.h"
#include "Transport/OS.h"


%statemachine_include_list%

namespace %service_namespace%
{

class DllExport %service_name% : public JTS::Service
{
public:
	%service_name%();
	virtual ~%service_name%();
	void init_service(std::shared_ptr<iop::Component> cmp, JTS::JausRouter* jausRouter, JTS::Service* parentService);

	%parent_service_type% getParent();

	virtual bool processTransitions(JTS::InternalEvent* ie);
	virtual bool defaultTransitions(JTS::InternalEvent* ie);

	// FSMs are public so that children services can access them
%statemachine_variable_list%	

protected:
	%parent_service_type% pParentService;
	virtual void run();

};

}

#endif // %service_name_allcaps%_H
