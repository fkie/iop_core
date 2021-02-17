%copyright%

#ifndef %statemachine_name_allcaps%_H
#define %statemachine_name_allcaps%_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "%service_namespace%/Messages/MessageSet.h"
#include "%service_namespace%/InternalEvents/InternalEventsSet.h"

%transport_class_aliases%
%parent_fsm_includes%

#include "%statemachine_name%_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>


namespace %service_namespace%
{

class DllExport %statemachine_name% : public JTS::StateMachine
{
public:
	%statemachine_name%(%parent_fsm_arguments%);
	virtual ~%statemachine_name%();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
%setup_iop_configuration_h%

	/// Action Methods
%action_method_declarations%

	/// Guard Methods
%guard_method_declarations%

	%statemachine_name%Context *context;
	
protected:

	/// References to parent FSMs
%parent_fsm_references%
%parent_fsm_iop_args%
};

}

#endif // %statemachine_name_allcaps%_H
