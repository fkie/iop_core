uint8 GRANTED         = 0
uint8 NOT_AVAILABLE   = 1
uint8 TIMEOUT         = 2
uint8 DENIED          = 3
uint8 QUEUED          = 4
uint8 DEFERRED        = 5
uint8 INSUFFICIENT_AUTHORITY  = 6
uint8 WAIT            = 7

# reponse code
uint8 code

# IOP request ID. This is the same as in the request.
uint8 request_id

# the address of component with EnhancendAccessControl which sent a request for this response
iop_msgs_fkie/JausAddress component