# the address of the controlled service
iop_msgs_fkie/JausAddress addr_control

# the uri of the remote service
string uri

# current access state
uint8 ACCESS_STATE_NOT_AVAILABLE          = 0
uint8 ACCESS_STATE_NOT_CONTROLLED         = 1
uint8 ACCESS_STATE_CONTROL_RELEASED       = 2
uint8 ACCESS_STATE_CONTROL_ACCEPTED       = 3
uint8 ACCESS_STATE_TIMEOUT                = 4
uint8 ACCESS_STATE_INSUFFICIENT_AUTHORITY = 5
uint8 ACCESS_STATE_MONITORING             = 6
uint8 access_state

uint8 authority
