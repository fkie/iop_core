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
#include "fkie_iop_component/iop_component.h"

#include <fkie_iop_builder/ros_init.h>

int main(int argc, char *argv[])
{
	// Instantiate the component and start it.
	try
	{
		iop::Component* cmpt = iop::ros_init<iop::Component>(argc, argv, "iop_component_default", 126, 0x40, 81);

		// Wait until signaled to exit
		//    exit_signal.wait();
		ros::spin();
		// Shutdown the component and threads
		cmpt->shutdown_component();

		// Give a little time for proper shutdown
		DeVivo::Junior::JrSleep(100);

		// Free the component
		delete cmpt;
	} catch (std::runtime_error &e) {
		ROS_ERROR("Failed to start IOP component: %s", e.what());
		exit(1);
	}
}
