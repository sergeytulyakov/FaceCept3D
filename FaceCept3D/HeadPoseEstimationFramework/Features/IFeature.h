#pragma once

#ifndef I_FEATURE_H
#define I_FEATURE_H

#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>

#include "Interface/IInterface.h"


namespace hpe
{
    template <typename PointType, typename NormalType>
    class IFeature : public IInterface
    {
        public:
            virtual cv::Mat Compute(typename pcl::PointCloud<PointType>::Ptr &cloud,
                                    PointType &point, NormalType &normal) = 0;

            virtual cv::Mat EmptyFeature() = 0;
    };
}

#endif //I_FEATURE_H