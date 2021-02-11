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

#pragma once

#include <string>
#include <vector>

namespace iop {

template <typename Separator>
auto split_aux(const std::string& value, Separator&& separator)
    -> std::vector<std::string>
{
    std::vector<std::string> result;
    std::string::size_type p = 0;
    std::string::size_type q;
    while ((q = separator(value, p)) != std::string::npos) {
        result.emplace_back(value, p, q - p);
        p = q + 1;
    }
    result.emplace_back(value, p);
    return result;
}

auto split(const std::string& value, char separator)
    -> std::vector<std::string>
{
    return split_aux(value,
        [=](const std::string& v, std::string::size_type p) {
            return v.find(separator, p);
        });
}

auto split(const std::string& value, const std::string& separators)
    -> std::vector<std::string>
{
    return split_aux(value,
        [&](const std::string& v, std::string::size_type p) {
            return v.find_first_of(separators, p);
        });
}

}