#include "HPESettings.h"

#include <Exception/HPEException.h>

#include <boost/property_tree/ini_parser.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

namespace hpe
{
    HPESettings::HPESettings(void)
    {
        Load("settings.ini");
    }

    HPESettings::HPESettings(std::string file)
    {
        Load(file);
    }

    HPESettings::HPESettings(bool load, std::string file)
    {
        if (load)
        {
            Load(file);
        }
    }

    HPESettings::~HPESettings(void)
    {
    }

    void HPESettings::Load(std::string file)
    {
        if (boost::filesystem::exists(file))
        {
            boost::property_tree::ini_parser::read_ini(file, m_propertyTree);
        }
        else
        {
            throw HPEException("Settings: file not found");
        }
    }

    std::string HPESettings::GetString(std::string key)
    {
        auto child = m_propertyTree.get_child_optional(key);
        if (!child)
        {
			std::string message = (boost::format("Settings: key %1% not found") % key).str();
            throw HPEException(message);
        }
        return m_propertyTree.get<std::string>(key);
    }


    std::vector<std::string> HPESettings::ReadList(std::string listFile)
    {
        if (boost::filesystem::exists(listFile) == false)
        {
            throw HPEException("std::vector<std::string> HPESettings::ReadList(std::string listFile) : listFile doesn't exist");
        }

        std::vector<std::string> result;
        std::ifstream stream(listFile);
        std::string line;
        while (std::getline(stream, line))
        {
            result.push_back(line);
        }

        return result;
    }

	std::vector<std::string> HPESettings::ReadStringList(std::string key)
	{
		return ReadList(GetString(key));
	}

}