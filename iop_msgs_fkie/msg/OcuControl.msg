# the address of the platform manager service. Only valid subsystem id is
# possible. 0.0.0 is an invalid id.
iop_msgs_fkie/JausAddress address

# OWNER                    255      'root# user concept, analogous to the root user of an OS.
# PRIMARY_OPERATOR         205-254  Only client role with mobility control. Can control/monitor any Component on the platform.
# ONBOARD_MOBILITY_DEVICE  155-204  Only onboard controller role with mobility control. Can control/monitor any Component on the platform.
# OPERATOR                 75-124   Client role that can control/monitor any Component on the platform except those contained under the Mobility Node.
# ONBOARD_DEVICE           25-74    Onboard controller role that can control/monitor any Component on the platform except those defined under the Mobility Node.
# MONITOR                  0        Can only monitor Components.
uint8 authority

# control the current access state
# ACCESS_CONTROL_ON_DEMAND    10    the slave ocu node can request access if it is needed
# ACCESS_CONTROL_REQUEST      11    force to request the access control
# ACCESS_CONTROL_RELEASE      12    release the control and don not try to request (also not on demand)
uint8 access_control

