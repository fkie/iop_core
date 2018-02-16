# do request handoff if request control was rejected
bool request

# IOP request ID. This is set by HandoffController and should be in the response.
uint8 request_id

# Authority of the source requesting the handoff.
uint8 authority_code

# Reason the handoff is requested.
string explanation

# the address of the component to control.
iop_msgs_fkie/JausAddress component

# the address of the ocu which send the request. For own requests it will be filled by HandoffController.
iop_msgs_fkie/JausAddress ocu