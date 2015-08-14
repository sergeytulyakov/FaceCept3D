#include "MapDataStorage.h"

namespace hpe
{
    void MapDataStorage::Set(std::string key, IDataObject::Ptr object)
    {
        m_storage[key] = object;
    }

    IDataObject::Ptr MapDataStorage::Get(std::string key)
    {
        return m_storage[key];
    }

    void MapDataStorage::Remove(std::string key)
    {
        auto it = m_storage.find(key);
        if (it != m_storage.end())
        {
            m_storage.erase(it);
        }
    }

}