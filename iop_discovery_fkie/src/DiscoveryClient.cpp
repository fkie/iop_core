/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */

#include <iostream>
#include <signal.h>
#include "Transport/OS.h"
#include "Iop_discovery_client_fkie.h"

#include <iop_builder_fkie/ros_init.h>

// Create a static signal to catch interrupts
static DeVivo::Junior::JrSignal exit_signal;
static void handle_exit_signal( int signum )
{
	exit_signal.signal();
	ros::shutdown();
}

int main(int argc, char *argv[])
{
	Iop_discovery_client_fkie* cmpt = iop::ros_init<Iop_discovery_client_fkie>(argc, argv, "iop_client_discovery", 127, 0x40, 200);

	// Catch exit signals
	signal( SIGINT, handle_exit_signal );
	signal( SIGTERM, handle_exit_signal );
	signal( SIGABRT, handle_exit_signal );

	// Start the component and the services
	cmpt->startComponent();

	// Wait until signaled to exit
	ros::spin();

	// Shutdown the component and threads
	cmpt->shutdownComponent();

	// Give a little time for proper shutdown
	DeVivo::Junior::JrSleep(100);

	// Free the component
	delete cmpt;
}

