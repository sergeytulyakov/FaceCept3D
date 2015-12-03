#pragma once
#pragma once

#ifndef I_PATCH_SAMPLER
#define I_PATCH_SAMPLER

#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <Interface/IInterface.h>
#include <Landmarks.h>

namespace hpe
{
    template <typename PointType>
    class IPatchSampler : public IInterface
    {
        public:
            virtual typename pcl::PointCloud<PointType>::Ptr SampleCloud(typename pcl::PointCloud<PointType>::Ptr &cloud) = 0;
            virtual typename pcl::PointCloud<PointType>::Ptr SampleCloud(typename pcl::PointCloud<PointType>::Ptr &cloud, const typename Common<PointType>::Landmarks &landmarks)
            {
                return SampleCloud(cloud);
            }
            virtual typename pcl::PointCloud<PointType>::Ptr SampleCloud(typename pcl::PointCloud<PointType>::Ptr &cloud, pcl::ModelCoefficients &coefficients)
            {
                coefficients.values.clear();
                return SampleCloud(cloud);
            }
            virtual typename pcl::PointCloud<PointType>::Ptr SampleCloud(typename pcl::PointCloud<PointType>::Ptr &cloud, const typename Common<PointType>::Landmarks &landmarks, pcl::ModelCoefficients &coefficients)
            {
                return SampleCloud(cloud, coefficients);
            }
    };
}

#endif //I_PATCH_SAMPLER