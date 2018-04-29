This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _iop_events_fkie:_ Events

The Events service implements the functionality to send registered events. The events are reports registered by plugins on load. Events supports periodic and on change notifications. Currently CommandEvents are not supported.

#### Parameter:

_events_timeout (int_, (Default: 1)

> Time period in minutes after which the event will be canceled if no update for event received. Zero disables the timeout.

#### Publisher:

> None

#### Subscriber:

> None

## _iop_events_fkie:_ EventsClient

This service register events on Events service and update it to avoid timeout or if change for event is needed. The received events forwarded to handler plugin which requested the report by event.

#### Parameter:

> None

#### Publisher:

> None

#### Subscriber:

> None
