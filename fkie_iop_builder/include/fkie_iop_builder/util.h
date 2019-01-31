/**
ROS/IOP Bridge
Copyright (c) 2018 Fraunhofer

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

#include <cmath>

#ifndef INCLUDE_FKIE_IOP_BUILDER_UTIL_H_
#define INCLUDE_FKIE_IOP_BUILDER_UTIL_H_

template<typename F>
F pround(const F& f, unsigned int decs=4)
{
	int i1 = floor(f);
	F rmnd = f - i1;
	int i2 = static_cast<int> (rmnd * pow(10, decs));
	F f1 = i2 / pow(10, decs);
	return i1 + f1;
}


#endif /* INCLUDE_FKIE_IOP_BUILDER_UTIL_H_ */
