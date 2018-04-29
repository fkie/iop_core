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

#include <iop_discovery_fkie/DiscoveryComponentList.h>
#include <algorithm>
#include <ros/ros.h>


using namespace iop;


DiscoveryComponentList::DiscoveryComponentList(unsigned int timeout) {
	p_timeout = timeout;
}

void DiscoveryComponentList::set_timeout(unsigned int timeout)
{
	p_timeout = timeout;
}

bool DiscoveryComponentList::add_service(JausAddress discovery_service, JausAddress component, std::string service_uri, unsigned char major_version, unsigned char minor_version)
{
	bool added = false;
	std::vector<DiscoveryComponent>& components = p_get_components(discovery_service);
	std::vector<DiscoveryComponent>::iterator it = std::find(components.begin(), components.end(), component);
	if (it == components.end()) {
		DiscoveryComponent dc(component);
		components.push_back(dc);
		added = components[components.size()-1].add_service(service_uri, major_version, minor_version);
	} else {
		added = it->add_service(service_uri, major_version, minor_version);
	}
	return added;
}

bool DiscoveryComponentList::update_ts(JausAddress discovery_service, JausAddress component)
{
	bool result = false;
	std::vector<DiscoveryComponent>& components = p_get_components(discovery_service);
	std::vector<DiscoveryComponent>::iterator it = std::find(components.begin(), components.end(), component);
	if (it != components.end()) {
		unsigned int now = ros::WallTime::now().sec;
		if (it->address == discovery_service) {
			it->ts_last_ident = now;
			result = true;
		} else {
			if (p_expired(it->ts_last_ident, now)) {
				ROS_DEBUG_NAMED("Discovery", "remove expired services of %s", it->address.str().c_str());
				components.erase(it);
			} else {
				it->ts_last_ident = now;
				result = true;
			}
		}
	}
	return result;
}

bool DiscoveryComponentList::update_ts(JausAddress discovery_service, unsigned short subsystem, unsigned char node)
{
	bool result = false;
	std::vector<DiscoveryComponent>& components = p_get_components(discovery_service);
	std::vector<DiscoveryComponent>::iterator it;
	for (it = components.begin(); it != components.end(); it++) {
		unsigned int now = ros::WallTime::now().sec;
		if (it->address == discovery_service) {
			it->ts_last_ident = now;
			result = true;
		} else if (it->address.getSubsystemID() == subsystem && (node == 255 || it->address.getNodeID() == node)) {
			if (p_expired(it->ts_last_ident, now)) {
				ROS_DEBUG_NAMED("Discovery", "remove expired services of %s", it->address.str().c_str());
				components.erase(it);
				it = components.begin();
			} else {
				it->ts_last_ident = now;
				result = true;
			}
		}
	}
	return result;
}

std::vector<DiscoveryComponent> DiscoveryComponentList::get_components(JausAddress discovery_service, unsigned short subsystem, unsigned char node, unsigned char component)
{
	std::vector<DiscoveryComponent>& components = p_get_components(discovery_service);
	std::vector<JausAddress> to_remove;
	std::vector<iop::DiscoveryComponent> result;
	std::vector<iop::DiscoveryComponent>::iterator itcmp;
	unsigned int now = ros::WallTime::now().sec;
	for (itcmp = components.begin(); itcmp != components.end(); itcmp++) {
		if (subsystem == 65535 || subsystem == itcmp->address.getSubsystemID()) {
			if (node == 255 || node == itcmp->address.getNodeID()) {
				if (component == 255 || component == itcmp->address.getComponentID()) {
					if (!p_expired(itcmp->ts_last_ident, now)) {
						result.push_back(*itcmp);
					} else {
						JausAddress adr(itcmp->address);
						// collect expired components
						to_remove.push_back(itcmp->address);
					}
				}
			}
		}
	}
	// remove expired components collected in previous step
	std::vector<JausAddress>::iterator itrm;
	for (itrm = to_remove.begin(); itrm != to_remove.end(); itrm++) {
		std::vector<DiscoveryComponent>::iterator itcmrm = std::find(components.begin(), components.end(), *itrm);
		if (itcmrm != components.end()) {
			components.erase(itcmrm);
		}
	}
	return result;
}

std::vector<DiscoveryComponent> DiscoveryComponentList::get_components(JausAddress discovery_service, std::string uri)
{
	std::vector<DiscoveryComponent> result;
	std::vector<DiscoveryComponent>& components = p_get_components(discovery_service);
	std::vector<iop::DiscoveryComponent>::iterator itcmp;
	for (itcmp = components.begin(); itcmp != components.end(); itcmp++) {
		if (itcmp->has_service(uri)) {
			result.push_back(*itcmp);
		}
	}
	return result;
}

void DiscoveryComponentList::remove_discovery_service(JausAddress addr)
{
	std::map<JausAddress, std::vector<DiscoveryComponent> >::iterator it;
	it = p_components.find(addr);
	if (it != p_components.end()) {
		p_components.erase(addr);
	}
}

std::vector<JausAddress> DiscoveryComponentList::get_discovery_services()
{
	std::vector<JausAddress> result;
	std::map<JausAddress, std::vector<DiscoveryComponent> >::iterator it;
	for (it = p_components.begin(); it != p_components.end(); ++it) {
		JausAddress adr(it->first);
		result.push_back(it->first);
	}
	return result;
}

bool DiscoveryComponentList::p_expired(unsigned int ts, unsigned int now)
{
	unsigned int mynow = now;
	if (mynow == 0) {
		mynow = ros::WallTime::now().sec;
	}
	return (p_timeout != 0 && ts != 0 && ts < mynow - p_timeout);
}

std::vector<DiscoveryComponent>& DiscoveryComponentList::p_get_components(JausAddress discovery_service)
{
	std::map<JausAddress, std::vector<DiscoveryComponent> >::iterator it;
	it = p_components.find(discovery_service);
	if (it == p_components.end()) {
		p_components[discovery_service] = std::vector<DiscoveryComponent>();
	}
	return p_components[discovery_service];
}
