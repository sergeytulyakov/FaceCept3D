#pragma once

#ifndef IDATAOBJECT_H
#define IDATAOBJECT_H

#include <Interface/IInterface.h>

#include <memory>

namespace hpe
{
    /**
     \class	IDataObject
    
     \brief	Interface to store the objects to put into IDataStorage.
			See subclasses for examples.
    
     \author	Sergey
     \date	8/11/2015
     */

    class IDataObject : public IInterface
    {
        public:
            typedef std::shared_ptr<IDataObject> Ptr;
    };
}

#endif