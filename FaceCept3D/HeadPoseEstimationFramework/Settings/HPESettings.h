#pragma once

#ifndef HPESETTINGS_H
#define HPESETTINGS_H

#include <boost/property_tree/ptree.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <iostream>

namespace hpe
{
    /**
     \class	HPESettings
    
     \brief	This class is used to read *.ini files.
     */

    class HPESettings
    {
        public:
            HPESettings(void);
            HPESettings(std::string file);

            HPESettings(bool load, std::string file = "settings.ini");
            ~HPESettings(void);

            void Load(std::string file);

            std::string GetString(std::string key);

            /**
             \fn	template <typename T> T HPESettings::GetValue(std::string key)
            
             \brief	Return a value from ini file and convert it to T.
            
             \author	Sergey
             \date	8/11/2015
            
             \tparam	T	type of the returned value.
             \param	key		The key in the ini file.
            
             \return	The value.
             */

            template <typename T>
            T GetValue(std::string key)
            {
                std::string value = GetString(key);
                return boost::lexical_cast<T>(value);
            }

            /**
             \fn	template <typename T> std::vector<T> HPESettings::GetVector(std::string key)
            
             \brief	Similar to GetValue(...), but to return a vector of T.	
					In an ini an int vector should be stored as 1,2,3,4,5,6 (comma separated)
            
             \tparam	T	All vector values will be cast to T.
             \param	key	The key.
            
             \return	The vector of T.
             */

            template <typename T>
            std::vector<T> GetVector(std::string key)
            {
                std::string value = GetString(key);
                std::vector<std::string> strs;
                boost::split(strs, value, boost::is_any_of(","));
                std::vector<T> result;
                for (int i = 0; i < strs.size(); i++)
                {
                    try
                    {
                        T v = boost::lexical_cast<T>(strs[i]);
                        result.push_back(v);
                    }
                    catch (boost::bad_lexical_cast &ex)
                    {
                        std::cout << ex.what() << std::endl;
                    }
                }

                return result;
            }

            static std::vector<std::string> ReadList(std::string listFile);
			std::vector<std::string> ReadStringList(std::string key);

        private:
            boost::property_tree::ptree m_propertyTree;
    };
}

#endif

