#pragma once

#ifndef MAPDATASTORAGE_H
#define MAPDATASTORAGE_H

#include <DataObject/IDataStorage.h>
#include <map>

namespace hpe
{

    class MapDataStorage : public IDataStorage
    {
        public:
            void Set(std::string key, IDataObject::Ptr object);
            IDataObject::Ptr Get(std::string key);
            void Remove(std::string key);
        private:
            std::map<std::string, IDataObject::Ptr> m_storage;
    };

}

#endif