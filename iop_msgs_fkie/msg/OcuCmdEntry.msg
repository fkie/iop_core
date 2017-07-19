# The message is used by OCU GUI component to inform OCU clients what they should try to control.
# At minimum a valid subsystem id must be defined. If component id is 0, it means all components 
# should be controled. 
# 0.0.0 is an invalid id.
iop_msgs_fkie/JausAddress address

# the name of the subsystem, node or component (depend on the specified address)
string name

# This command is for OCU client with given JAUS Address. It is usefull, if more then one
# node can control available services (e.g. two PTU's)
iop_msgs_fkie/JausAddress ocu_client

# OWNER                    255      'root# user concept, analogous to the root user of an OS.
# PRIMARY_OPERATOR         205-254  Only client role with mobility control. Can control/monitor any Component on the platform.
# ONBOARD_MOBILITY_DEVICE  155-204  Only onboard controller role with mobility control. Can control/monitor any Component on the platform.
# OPERATOR                 75-124   Client role that can control/monitor any Component on the platform except those contained under the Mobility Node.
# ONBOARD_DEVICE           25-74    Onboard controller role that can control/monitor any Component on the platform except those defined under the Mobility Node.
# MONITOR                  0        Can only monitor Components.
uint8 authority

# control the current access state
# ACCESS_CONTROL_RELEASE      10    release the control and don not try to request
# ACCESS_CONTROL_MONITOR      11    do not request access control, only try to register the events with data
# ACCESS_CONTROL_REQUEST      12    force to request the access control
uint8 access_control

