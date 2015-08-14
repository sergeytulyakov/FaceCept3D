#pragma once

#ifndef IDATASTORAGE_H
#define IDATASTORAGE_H

#include <Interface/IInterface.h>
#include <DataObject/IDataObject.h>
#include <Exception/HPEException.h>

#include <string>

namespace hpe
{
    /**
     \class	IDataStorage
    
     \brief	One of the key classes to organize between communications processors. 
			Basically is a key-value storage. 
    
     \author	Sergey
     \date	8/11/2015
     */

    class IDataStorage : public IInterface
    {
        public:
            typedef std::shared_ptr<IDataStorage> Ptr;

            /**
             \fn	virtual void IDataStorage::Set(std::string key, IDataObject::Ptr object) = 0;
            
             \brief	Add object into storage with key
            
             \author	Sergey
             \date	8/11/2015
            
             \param	key   	Key
             \param	object	Object to add. The object should be contained in IDataObject
             */

            virtual void Set(std::string key, IDataObject::Ptr object) = 0;
            virtual IDataObject::Ptr Get(std::string key) = 0;
            virtual void Remove(std::string key) = 0;

            template<class T>
            std::shared_ptr<T> GetAndCast(std::string key)
            {
                IDataObject::Ptr obj = Get(key);
                return std::dynamic_pointer_cast<T>(obj);
            }

            template<class T>
            std::shared_ptr<T> GetAndCastNotNull(std::string key, std::string message = "Got nullptr from storage")
            {
                std::shared_ptr<T> ptr = GetAndCast<T>(key);
                if (ptr.get() == nullptr)
                {
                    throw HPEException(message);
                }
                return ptr;
            }

    };

}

#endif