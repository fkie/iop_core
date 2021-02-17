%copyright%

#include <pluginlib/class_list_macros.hpp>
#include <fkie_iop_component/iop_component.hpp>
#include "%service_namespace%/%service_name%.h"

using namespace JTS;

namespace %service_namespace%
{

%service_name%::%service_name%() : Service()
{
	p_initialized = false;
	pParentService = nullptr;
%service_assignment_list%
}

void %service_name%::init_service(std::shared_ptr<iop::Component> cmp, JTS::JausRouter* jausRouter, JTS::Service* parentService)
{
	if (!p_initialized) {
%statemachine_assignment_list%
	}
	p_initialized = true;
}


%service_name%::~%service_name%()
{
	if (p_initialized) {
%statemachine_destruction_list%
	}
}

%parent_service_type% %service_name%::getParent()
{
	return pParentService;
}

/**
 *	This is the function that will actually be run by the thread at start-up.
 *  We override it from EventReceiver in order to handle any entry
 *  actions defined by the initial state. 
 */
void %service_name%::run()
{
	
	/// Perform any entry actions specified by the start state.
%start_state_actions%
	
	/// Kick-off the receive loop...
	EventReceiver::run();
}

/**
 *	This is the function that will process an event either generated
 *  by the service, sent to it by another service on the same component,
 *  or as a message sent by a different component. 
 */
bool %service_name%::processTransitions(InternalEvent* ie)
{
        bool done = false;

   // Since this function can be called from multiple threads,
   // we use a mutex to ensure only one state transition is
   // active at a time.
   mutex.lock();

			// Invoke the FSM transition for this event.
%transition_calls%			 
leave:
   mutex.unlock();
   return done;
}


/**
 *	This is the function that will check for default transitions if
 *  no other transitions were satisfied. 
 */
bool %service_name%::defaultTransitions(InternalEvent* ie)
{
   bool done = false;

   // Since this function can be called from multiple threads,
   // we use a mutex to ensure only one state transition is
   // active at a time.
   mutex.lock();

			// Invoke the FSM transition for this event.
%default_transition_calls%	
leave:
   mutex.unlock();
   return done;
}


}

PLUGINLIB_EXPORT_CLASS(%service_namespace%::%statemachine_name_prefix%, JTS::Service)