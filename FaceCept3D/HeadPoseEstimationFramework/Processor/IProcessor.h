#pragma once

#ifndef IPROCESSOR_H
#define IPROCESSOR_H

#include <Interface/IInterface.h>
#include <DataObject/IDataStorage.h>
#include <memory>

namespace hpe
{
    class IProcessor : public IInterface
    {
        public:
            typedef std::shared_ptr<IProcessor> Ptr;

            virtual void Process(IDataStorage::Ptr dataStorage) = 0;
            virtual void Init() {};
            virtual void Cleanup() {};
    };
}

#endif