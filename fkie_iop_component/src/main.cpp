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
#include <rclcpp/rclcpp.hpp>
#include "Transport/OS.h"
#include "fkie_iop_component/ros_node.hpp"
#include "fkie_iop_component/iop_component.h"


int main(int /* argc */, char* /*argv*/ [] )
{
    // Instantiate the component and start it.
//     try
//     {
        auto rosnode = std::make_shared<iop::RosNode>("iop_component_default", "");
        rosnode->init_component(126, 0x40, 81);

        // Wait until signaled to exit
        //    exit_signal.wait();
        rclcpp::spin(rosnode);
        //ros::spin();
        // Shutdown the component and threads
        rosnode->get_component()->shutdown_component();
        rclcpp::shutdown();

        // Give a little time for proper shutdown
        DeVivo::Junior::JrSleep(100);
//     } catch (std::runtime_error &e) {
//         RCLCPP_ERROR("Failed to start IOP component: %s", e.what());
//         exit(1);
//     }
}
