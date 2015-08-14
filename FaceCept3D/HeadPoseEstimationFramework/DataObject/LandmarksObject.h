#pragma once

#ifndef LANDMARKSOBJECT_H
#define LANDMARKSOBJECT_H

#include <DataObject/IDataObject.h>

#include <Landmarks.h>

namespace hpe
{
    template <class PointType>
    class LandmarksObject : public IDataObject
    {
        public:
            typedef std::shared_ptr<LandmarksObject> Ptr;

            typename Common<PointType>::Landmarks landmarks;
    };
}

#endif