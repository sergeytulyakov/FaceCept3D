#pragma once

#ifndef I_FEATURE_CALCULATOR_H
#define I_FEATURE_CALCULATOR_H

#include "Interface/IInterface.h"

namespace hpe
{
    template <typename PointType>
    class IFeatureCalculator : public IInterface
    {
        public:
            virtual cv::Mat GetFeatures(typename pcl::PointCloud<PointType>::Ptr &cloud) = 0;
    };
}


#endif //I_FEATURE_CALCULATOR_H