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
#include "fkie_iop_component/iop_component.hpp"


int main(int argc, char* argv [] )
{
    // Instantiate the component and start it.
    rclcpp::init(argc, argv);
    auto component = std::make_shared<iop::Component>("iop_component_default", "");
    component->init(126, 0x40, 81);
    rclcpp::spin(component);
    // Shutdown the component and threads
    component->shutdown_component();
    rclcpp::shutdown();
    // Give a little time for proper shutdown
    DeVivo::Junior::JrSleep(100);
}
