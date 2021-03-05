/*!
 ***********************************************************************
 * @file      XmlConfig.cpp
 * @author    Dave Martin, DeVivo AST, Inc.
 * @date      2008/03/03
 *
 *  Copyright (C) 2008. DeVivo AST, Inc
 *
 *  This file is part of Jr Middleware.
 *
 *  Jr Middleware is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Jr Middleware is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with Jr Middleware.  If not, see <http://www.gnu.org/licenses/>.
 *
 ************************************************************************
 */
#include "Transport/XmlConfig.h"
using namespace tinyxml2;
using namespace DeVivo::Junior;

XmlConfig::XmlConfig()
{

}

XmlConfig::~XmlConfig()
{

}

ConfigData::ConfigError XmlConfig::parseFile(const  std::string& filename)
{
	XMLError load_err = doc.LoadFile(filename.c_str());
	if (load_err != XML_SUCCESS)
	{
		JrError << "Failed to parse config file: " << filename << "(Error: " << load_err << ")"<< std::endl;

		// Cast the TinyXML errors to our enum
		if (load_err == XML_ERROR_FILE_NOT_FOUND)
			return FileNotFound;
		return InvalidFile;
	}
	return Ok;
}

// getValue implementations make use of templated accessor
ConfigData::ConfigError XmlConfig::getValue(std::string& value,
			  					 const std::string& attribute,
								 const std::string& element,
								 int index)
{
	return lookupValue(value, attribute, element, index);
}

ConfigData::ConfigError XmlConfig::getValue(int& value,
			  					 const std::string& attribute,
								 const std::string& element,
								 int index)
{
	return lookupValue(value, attribute, element, index);
}

ConfigData::ConfigError XmlConfig::getValue(unsigned int& value,
  					 const std::string& attribute,
					 const std::string& element,
					 int index)
{
	return lookupValue(value, attribute, element, index);
}

ConfigData::ConfigError XmlConfig::getValue(short& value,
  					 const std::string& attribute,
					 const std::string& element,
					 int index)
{
	return lookupValue(value, attribute, element, index);
}

ConfigData::ConfigError XmlConfig::getValue(unsigned short& value,
  					 const std::string& attribute,
					 const std::string& element,
					 int index)
{
	return lookupValue(value, attribute, element, index);
}

ConfigData::ConfigError XmlConfig::getValue(char& value,
		  					 const std::string& attribute,
							 const std::string& element,
							 int index)
{
	return lookupValue(value, attribute, element, index);
}

ConfigData::ConfigError XmlConfig::getValue(unsigned char& value,
		  					 const std::string& attribute,
							 const std::string& element,
							 int index)
{
	return lookupValue(value, attribute, element, index);
}

ConfigData::ConfigError XmlConfig::getValue(bool& value,
		  					 const std::string& attribute,
							 const std::string& element,
							 int index)
{
	return lookupValue(value, attribute, element, index);
}

ConfigData::ConfigError XmlConfig::getValue(double& value,
		  					 const std::string& attribute,
							 const std::string& element,
							 int index)
{
	return lookupValue(value, attribute, element, index);
}

template <typename T>
ConfigData::ConfigError XmlConfig::lookupValue(T& value,
											   const std::string& attribute,
											   const std::string& element,
											   int index)

{
	// Get the first occurrence of the requested element
	bool element_found = false;
	XMLElement* ele = doc.FirstChildElement("JrXmlConfig");
	if (ele != NULL)
	{
		ele = ele->FirstChildElement(element.c_str());
		// Loop through the index values to find the requested element number
		while (ele != NULL && index > 0 && !element_found)
		{
			ele = ele->NextSiblingElement(element.c_str());
			if (ele != NULL)
			{
				element_found = true;
			}
			index--;
		}
	}
	if (ele == NULL)
	{
		JrWarn_ << "Failed to find configuration element: " << element << ", use default " << attribute << ": " << value << "\n";
		return ValueNotFound;
	}

	// Now that we have the right element, pull the requested attribute.
	const char *attr_value;
	attr_value = ele->Attribute(attribute.c_str());
	if (attr_value == 0)
	{
		JrWarn_ << "Failed to find configuration attribute: " << attribute  << ", use default value: " << value << "\n";
		return ValueNotFound;
	}
	std::stringstream sstream( attr_value );
	sstream >> value;
	if ( sstream.fail() ) {
		JrWarn_ << "Failed to read configuration attribute: " << attribute  << ", use default value: '" << value << "'\n";
		return ValueNotFound;
	}
	// Success!
	JrInfo_ << "Found config value: " << attribute << " = " << value << std::endl;
	return Ok;
}


StringList XmlConfig::getAttributes(std::string element)
{
	StringList ret;
	// Get the first occurrence of the requested element
	XMLElement* ele = doc.FirstChildElement("JrXmlConfig");
	if (ele != NULL)
	{
		ele = ele->FirstChildElement(element.c_str());
	}
	if (ele == NULL)
		return ret;
	// Walk through the attributes, returning a string for each
	for (const XMLAttribute* att = ele->FirstAttribute(); att != NULL; att = att->Next())
	{
		JrDebug << "Found attribute: " << att->Name() << "\n";
		ret.push_back(att->Name());
	}
	return ret;
}
