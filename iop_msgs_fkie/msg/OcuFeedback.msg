# the address of the reporter of this message
iop_msgs_fkie/JausAddress reporter
# if this value is not 0 or 65535 the client is speciefied to control or listen to the specified subsystem
uint16 subsystem_restricted
# If set to True the client does not allow to control the subsystem
bool only_monitor
# If set to True the client contains the HandoffController service
bool handoff_supported
# the list of supported services and they current state
iop_msgs_fkie/OcuServiceInfo[] services
