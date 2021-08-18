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

#ifndef IOP_STRING
#define IOP_STRING

#include <string>
#include <vector>

namespace iop {

template <typename Separator>
auto split_aux(const std::string& value, Separator&& separator, std::string::size_type max_count=0)
    -> std::vector<std::string>
{
    std::vector<std::string> result;
    std::string::size_type p = 0;
    std::string::size_type q;
    std::string::size_type count = 1;
    while ((q = separator(value, p)) != std::string::npos && (max_count == 0 || count < max_count)) {
        result.emplace_back(value, p, q - p);
        p = q + 1;
        count++;
    }
    result.emplace_back(value, p);
    return result;
}

inline auto split(const std::string& value, char separator, std::string::size_type max_count=0)
    -> std::vector<std::string>
{
    return split_aux(value,
        [=](const std::string& v, std::string::size_type p) {
            return v.find(separator, p);
        }, max_count);
}

inline auto split(const std::string& value, const std::string& separators, std::string::size_type max_count=0)
    -> std::vector<std::string>
{
    return split_aux(value,
        [&](const std::string& v, std::string::size_type p) {
            return v.find_first_of(separators, p);
        }, max_count);
}

inline std::string& ltrim(std::string& str, const std::string& chars="\t\n\v\f\r ")
{
    str.erase(0, str.find_first_not_of(chars));
    return str;
}
 
inline std::string& rtrim(std::string& str, const std::string& chars="\t\n\v\f\r ")
{
    str.erase(str.find_last_not_of(chars) + 1);
    return str;
}
 
inline std::string& trim(std::string& str, const std::string& chars="\t\n\v\f\r ")
{
    return ltrim(rtrim(str, chars), chars);
}

inline bool stob(const std::string& v)
{
    return !v.empty () &&
        (strcasecmp (v.c_str (), "true") == 0 ||
         atoi (v.c_str ()) != 0);
}

}

#endif